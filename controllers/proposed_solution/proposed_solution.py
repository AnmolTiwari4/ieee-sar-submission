from controller import Robot
import json
import cv2
import numpy as np
import math
import heapq
import os
import random
from steering_pid import PIDSteeringController

# Initialize your steering engine right after setup
steering_engine = PIDSteeringController(kp=3.0, max_speed=6.0)

# =========================================================================
# 1. INITIALIZATION & HARDWARE SETUP
# =========================================================================
rosbot = Robot()
timestep = int(rosbot.getBasicTimeStep())
robot_id = rosbot.getName()

# Seed the random number generator using the robot's name 
# This guarantees robot1 and robot2 pick completely different search routes!
random.seed(robot_id)

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
# 2. HACKATHON COMMUNICATIONS & LOGGING PIPELINE
# =========================================================================
supervisor_emitter = rosbot.getDevice("supervisor emitter")
supervisor_emitter.setChannel(43)

def report_victim(estimated_x, estimated_y, confidence=0.95):
    """Sends JSON payload on Channel 43 AND writes to local CSV log."""
    # 1. Official JSON Payload to Supervisor
    message_dict = {
        "timestamp": rosbot.getTime(),
        "robot_id": robot_id,
        "position": [estimated_x, estimated_y, 0.0],
        "victim_found": True,
        "victim_confidence": confidence 
    }
    supervisor_emitter.send(json.dumps(message_dict).encode('utf-8'))
    print(f"[{robot_id}] VICTIM REPORTED TO JUDGES AT ({estimated_x:.2f}, {estimated_y:.2f})")
    
    # 2. Automated CSV Logging Deliverable
    log_dir = 'sim_logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
        
    csv_file = os.path.join(log_dir, 'victim_location_estimates.csv')
    
    if not os.path.isfile(csv_file):
        with open(csv_file, 'w') as f:
            f.write("Timestamp,Robot_ID,Estimated_X,Estimated_Y,Confidence\n")
            
    with open(csv_file, 'a') as f:
        f.write(f"{rosbot.getTime()},{robot_id},{estimated_x},{estimated_y},{confidence}\n")

# =========================================================================
# 3. GLOBAL MAP & SCALING VARIABLES
# =========================================================================
try:
    global_map = cv2.imread('sim_logs/map_estimate.png', cv2.IMREAD_GRAYSCALE)
    if global_map is None: 
        raise FileNotFoundError
    print(f"[{robot_id}] Global map loaded successfully.")
except:
    print(f"[{robot_id}] WARNING: Map not found. Defaulting to blank grid.")
    global_map = np.ones((600, 600), dtype=np.uint8) * 255

# --- THE SCALE PATCH ---
# Change this number if the judges use the medium (20.0) or large (40.0) maps
ARENA_SIZE = 10.0 
GRID_SIZE = 60

current_x = -0.375
current_y = 0.375 if robot_id == "robot1" else 0.0
current_heading = 0.0
last_encoder_values = {"fl": 0.0, "fr": 0.0}

WHEEL_RADIUS = 0.0425
WHEEL_BASE = 0.192

# =========================================================================
# 4. LOGIC ENGINES (Odometry, Vision, Pathfinding)
# =========================================================================

def update_odometry():
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

def world_to_grid(wx, wy):
    """Dynamic translation using ARENA_SIZE instead of hardcoded 10.0"""
    offset = ARENA_SIZE / 2.0
    gx = int(np.clip((wx + offset) / ARENA_SIZE * GRID_SIZE, 0, GRID_SIZE - 1))
    gy = int(np.clip((offset - wy) / ARENA_SIZE * GRID_SIZE, 0, GRID_SIZE - 1))
    return (gx, gy)

def grid_to_world(gx, gy):
    """Dynamic translation using ARENA_SIZE instead of hardcoded 10.0"""
    offset = ARENA_SIZE / 2.0
    wx = (gx / float(GRID_SIZE)) * ARENA_SIZE - offset
    wy = offset - (gy / float(GRID_SIZE)) * ARENA_SIZE
    return (wx, wy)

def a_star_pathfind(start_world, target_world):
    small_map = cv2.resize(global_map, (GRID_SIZE, GRID_SIZE))
    kernel = np.ones((3, 3), np.uint8)
    inflated_map = cv2.erode(small_map, kernel)
    obstacle_grid = inflated_map < 127
    
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
                if obstacle_grid[ny, nx]: continue
                
                tentative_g = g_score[current] + math.hypot(dx, dy)
                neighbor = (nx, ny)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = math.hypot(goal[0] - nx, goal[1] - ny)
                    heapq.heappush(open_set, (tentative_g + h, neighbor))

    print(f"[{robot_id}] WARNING: A* found no valid path. Proceeding blind.")
    return [target_world]

def generate_safe_waypoints(map_array, num_points=4):
    """Automatically scans the map for open floor space and generates safe coordinates."""
    small_map = cv2.resize(map_array, (GRID_SIZE, GRID_SIZE))
    
    # A thick kernel ensures we pick targets FAR away from the walls
    kernel = np.ones((5, 5), np.uint8) 
    inflated_map = cv2.erode(small_map, kernel)
    
    # Find all Y, X indices of safe, open floor pixels
    safe_y, safe_x = np.where(inflated_map > 127)
    
    waypoints = []
    if len(safe_x) > 0:
        indices = list(range(len(safe_x)))
        random.shuffle(indices) # Seeded by robot_id above!
        
        for idx in indices[:num_points]:
            gx, gy = safe_x[idx], safe_y[idx]
            # Use dynamic translation
            wx, wy = grid_to_world(gx, gy)
            waypoints.append((wx, wy))
    else:
        waypoints = [(0.0, 0.0)]
        
    return waypoints

# =========================================================================
# 5. MAIN AUTONOMY LOOP
# =========================================================================

print(f"[{robot_id}] Systems green. Engaging autonomous search.")

# Auto-generate unique, map-safe sweep patterns for each robot
search_waypoints = generate_safe_waypoints(global_map, num_points=6)
print(f"[{robot_id}] Generated Safe Sweep Targets: {search_waypoints}")

# Pop the first target and calculate the initial path
current_target = search_waypoints.pop(0)
current_path = a_star_pathfind((current_x, current_y), current_target)

while rosbot.step(timestep) != -1:
    
    update_odometry()
    
    # --- WAYPOINT QUEUE LOGIC ---
    distance_to_target = math.hypot(current_target[0] - current_x, current_target[1] - current_y)

    if distance_to_target < 0.2:
        if len(search_waypoints) > 0:
            current_target = search_waypoints.pop(0)
            print(f"[{robot_id}] Target reached. Rerouting to: {current_target}")
            current_path = a_star_pathfind((current_x, current_y), current_target)
        else:
            print(f"[{robot_id}] Sector sweep complete. Holding position.")
            for motor in motors.values():
                motor.setVelocity(0.0)
            continue 
    
    # --- VICTIM DETECTION ---
    if detect_victim():
        for motor in motors.values(): 
            motor.setVelocity(0.0)
        report_victim(current_x, current_y)
    
    # --- PID STEERING ---
    left_speed, right_speed = steering_engine.calculate_speeds(
        current_x, current_y, current_heading, current_path
    )
    
    motors["fl"].setVelocity(left_speed)
    motors["rl"].setVelocity(left_speed)
    motors["fr"].setVelocity(right_speed)
    motors["rr"].setVelocity(right_speed)