from controller import Robot
import json
import cv2
import numpy as np
import math
import heapq
from steering_pid import PIDSteeringController

# Initialize your steering engine right after setup
steering_engine = PIDSteeringController(kp=3.0, max_speed=6.0)

# =========================================================================
# 1. INITIALIZATION & HARDWARE SETUP
# =========================================================================
rosbot = Robot()
timestep = int(rosbot.getBasicTimeStep())
robot_id = rosbot.getName()

# --- Motors ---
motors = {
    "fl": rosbot.getDevice("fl_wheel_joint"),
    "fr": rosbot.getDevice("fr_wheel_joint"),
    "rl": rosbot.getDevice("rl_wheel_joint"),
    "rr": rosbot.getDevice("rr_wheel_joint")
}
for motor in motors.values():
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# --- Sensors ---
lidar = rosbot.getDevice("laser")
lidar.enable(timestep)
lidar.enablePointCloud()

camera = rosbot.getDevice("camera rgb")
camera.enable(timestep)

compass = rosbot.getDevice("imu compass")
compass.enable(timestep)

encoders = {
    "fl": rosbot.getDevice("front left wheel motor sensor"),
    "fr": rosbot.getDevice("front right wheel motor sensor"),
    "rl": rosbot.getDevice("rear left wheel motor sensor"),
    "rr": rosbot.getDevice("rear right wheel motor sensor")
}
for encoder in encoders.values():
    encoder.enable(timestep)

# =========================================================================
# 2. HACKATHON COMMUNICATIONS PIPELINE
# =========================================================================
supervisor_emitter = rosbot.getDevice("supervisor emitter")
supervisor_emitter.setChannel(43)

def report_victim(estimated_x, estimated_y, confidence=0.95):
    """Sends the official JSON payload to score points on Channel 43."""
    message_dict = {
        "timestamp": rosbot.getTime(),
        "robot_id": robot_id,
        "position": [estimated_x, estimated_y, 0.0],
        "victim_found": True,
        "victim_confidence": confidence 
    }
    supervisor_emitter.send(json.dumps(message_dict).encode('utf-8'))
    print(f"[{robot_id}] VICTIM REPORTED TO JUDGES AT ({estimated_x:.2f}, {estimated_y:.2f})")

squad_receiver = rosbot.getDevice("robot to robot receiver")
squad_receiver.enable(timestep)
squad_emitter = rosbot.getDevice("robot to robot emitter")

def broadcast_target(target_x, target_y):
    """Tells the teammate robot which grid coordinate we are claiming."""
    message = {"robot_id": robot_id, "action": "claiming_target", "target": [target_x, target_y]}
    squad_emitter.send(json.dumps(message).encode('utf-8'))

# =========================================================================
# 3. GLOBAL MAP & STATE VARIABLES
# =========================================================================
try:
    global_map = cv2.imread('sim_logs/map_estimate.png', cv2.IMREAD_GRAYSCALE)
    if global_map is None: 
        raise FileNotFoundError
    print(f"[{robot_id}] Global map loaded successfully.")
except:
    print(f"[{robot_id}] WARNING: Map not found. Defaulting to blank grid.")
    global_map = np.ones((600, 600), dtype=np.uint8) * 255

# Robot State
current_x = -0.375
current_y = 0.375 if robot_id == "robot1" else 0.0
current_heading = 0.0
last_encoder_values = {"fl": 0.0, "fr": 0.0}

# Constants for ROSbot
WHEEL_RADIUS = 0.0425
WHEEL_BASE = 0.192

# =========================================================================
# 4. LOGIC ENGINES
# =========================================================================

def update_odometry():
    """Calculates position using dead reckoning (Encoders + Compass)."""
    global current_x, current_y, current_heading, last_encoder_values
    
    fl_val = encoders["fl"].getValue()
    fr_val = encoders["fr"].getValue()
    
    dist_left = (fl_val - last_encoder_values["fl"]) * WHEEL_RADIUS
    dist_right = (fr_val - last_encoder_values["fr"]) * WHEEL_RADIUS
    
    last_encoder_values["fl"] = fl_val
    last_encoder_values["fr"] = fr_val
    
    dist_center = (dist_left + dist_right) / 2.0
    
    compass_vals = compass.getValues()
    current_heading = math.atan2(compass_vals[0], compass_vals[1])
    
    current_x += dist_center * math.cos(current_heading)
    current_y += dist_center * math.sin(current_heading)

def detect_victim():
    """Uses OpenCV HSV masking, contour geometry, and LiDAR depth to confirm a victim."""
    raw_img = camera.getImage()
    if not raw_img: 
        return False
    
    img_array = np.frombuffer(raw_img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    frame_bgr = img_array[:, :, :3]
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    
    lower_color = np.array([5, 120, 120])
    upper_color = np.array([25, 255, 255])
    
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        area = cv2.contourArea(contour)
        
        if area > 1200: 
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            
            if 0.2 < aspect_ratio < 1.5:
                depth_data = lidar.getRangeImage()
                center_depth = depth_data[len(depth_data) // 2]
                
                if center_depth < 1.0:
                    return True
            
    return False

def a_star_pathfind(start_world, target_world):
    """
    Production A* Algorithm. 
    Downsamples the map, inflates walls, and translates Webots meters to matrix indices.
    """
    GRID_SIZE = 60
    
    small_map = cv2.resize(global_map, (GRID_SIZE, GRID_SIZE))
    
    kernel = np.ones((3, 3), np.uint8)
    inflated_map = cv2.erode(small_map, kernel)
    obstacle_grid = inflated_map < 127
    
    def world_to_grid(wx, wy):
        gx = int(np.clip((wx + 5.0) / 10.0 * GRID_SIZE, 0, GRID_SIZE - 1))
        gy = int(np.clip((5.0 - wy) / 10.0 * GRID_SIZE, 0, GRID_SIZE - 1))
        return (gx, gy)

    def grid_to_world(gx, gy):
        wx = (gx / float(GRID_SIZE)) * 10.0 - 5.0
        wy = 5.0 - (gy / float(GRID_SIZE)) * 10.0
        return (wx, wy)

    start = world_to_grid(start_world[0], start_world[1])
    goal = world_to_grid(target_world[0], target_world[1])
    
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]

    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(grid_to_world(current[0], current[1]))
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if obstacle_grid[ny, nx]:
                    continue
                
                tentative_g = g_score[current] + math.hypot(dx, dy)
                neighbor = (nx, ny)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    
                    h = math.hypot(goal[0] - nx, goal[1] - ny)
                    heapq.heappush(open_set, (tentative_g + h, neighbor))

    print(f"[{robot_id}] WARNING: A* found no valid path. Proceeding blind.")
    return [target_world]

# =========================================================================
# 5. MAIN AUTONOMY LOOP
# =========================================================================

print(f"[{robot_id}] Systems green. Engaging autonomous search.")

# UPGRADE 2: Define separate sweep patterns so they don't crash into each other
if robot_id == "robot1":
    search_waypoints = [
        (0.5, 0.5),    # robot1 takes the left side
        (1.5, -1.0)
    ]
else:
    search_waypoints = [
        (3.0, -2.0),   # robot2 takes the right side
        (2.5, 1.5)
    ]

# Pop the first target and calculate the initial path
current_target = search_waypoints.pop(0)
current_path = a_star_pathfind((current_x, current_y), current_target)

while rosbot.step(timestep) != -1:
    
    update_odometry()
    
    # ---------------------------------------------------------
    # WAYPOINT QUEUE LOGIC
    # ---------------------------------------------------------
    distance_to_target = math.hypot(current_target[0] - current_x, current_target[1] - current_y)

    if distance_to_target < 0.2:
        if len(search_waypoints) > 0:
            current_target = search_waypoints.pop(0)
            print(f"[{robot_id}] Waypoint reached. Calculating path to next zone: {current_target}")
            current_path = a_star_pathfind((current_x, current_y), current_target)
        else:
            print(f"[{robot_id}] Search pattern complete. Holding position.")
            for motor in motors.values():
                motor.setVelocity(0.0)
            continue 
    
    # ---------------------------------------------------------
    # VICTIM DETECTION & STEERING
    # ---------------------------------------------------------
    if detect_victim():
        for motor in motors.values(): 
            motor.setVelocity(0.0)
        report_victim(current_x, current_y)
    
    # Pass your current position, heading, and path into your separate steering file
    left_speed, right_speed = steering_engine.calculate_speeds(
        current_x, current_y, current_heading, current_path
    )
    
    # Apply the calculated speeds to the physical motors
    motors["fl"].setVelocity(left_speed)
    motors["rl"].setVelocity(left_speed)
    motors["fr"].setVelocity(right_speed)
    motors["rr"].setVelocity(right_speed)