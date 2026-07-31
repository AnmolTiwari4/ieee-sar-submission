from controller import Robot
import json
import cv2
import numpy as np
import math
import heapq
import os
import random
from steering_pid import PIDSteeringController

# Initialize steering engine
steering_engine = PIDSteeringController(kp=3.0, max_speed=6.0)

# =========================================================================
# 1. INITIALIZATION & UNIVERSAL MAP LOADING
# =========================================================================
rosbot = Robot()
timestep = int(rosbot.getBasicTimeStep())
robot_id = rosbot.getName()

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
# 2. COMMUNICATIONS & LOGGING
# =========================================================================
supervisor_emitter = rosbot.getDevice("supervisor emitter")
supervisor_emitter.setChannel(43)

def report_victim(estimated_x, estimated_y, confidence=0.95):
    message_dict = {
        "timestamp": rosbot.getTime(),
        "robot_id": robot_id,
        "position": [estimated_x, estimated_y, 0.0],
        "victim_found": True,
        "victim_confidence": confidence 
    }
    supervisor_emitter.send(json.dumps(message_dict).encode('utf-8'))
    print(f"[{robot_id}] VICTIM REPORTED AT ({estimated_x:.2f}, {estimated_y:.2f})")
    
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
# 3. UNIVERSAL MAP & AUTO-SCALING ENGINE
# =========================================================================
try:
    global_map = cv2.imread('sim_logs/map_estimate.png', cv2.IMREAD_GRAYSCALE)
    if global_map is None: 
        raise FileNotFoundError
    print(f"[{robot_id}] Universal map loaded successfully.")
except:
    print(f"[{robot_id}] WARNING: Map missing. Generating universal backup grid.")
    global_map = np.ones((600, 600), dtype=np.uint8) * 255
    cv2.rectangle(global_map, (30, 30), (570, 570), 0, 15)

ARENA_SIZE = 10.0 
GRID_SIZE = 60

current_x = -0.375
current_y = 0.375 if robot_id == "robot1" else 0.0
current_heading = 0.0
last_encoder_values = {"fl": 0.0, "fr": 0.0}

WHEEL_RADIUS = 0.0425

# =========================================================================
# 4. LOGIC ENGINES (Universal Coordinate & Pathfinding)
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
        if cv2.contourArea(contour) > 1200:
            depth_data = lidar.getRangeImage()
            if depth_data[len(depth_data) // 2] < 1.0:
                return True
    return False

def world_to_grid(wx, wy):
    offset = ARENA_SIZE / 2.0
    gx = int(np.clip((wx + offset) / ARENA_SIZE * GRID_SIZE, 0, GRID_SIZE - 1))
    gy = int(np.clip((offset - wy) / ARENA_SIZE * GRID_SIZE, 0, GRID_SIZE - 1))
    return (gx, gy)

def grid_to_world(gx, gy):
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

    return [target_world]

def generate_safe_waypoints(map_array, num_points=4):
    small_map = cv2.resize(map_array, (GRID_SIZE, GRID_SIZE))
    kernel = np.ones((5, 5), np.uint8) 
    inflated_map = cv2.erode(small_map, kernel)
    safe_y, safe_x = np.where(inflated_map > 127)
    
    waypoints = []
    if len(safe_x) > 0:
        indices = list(range(len(safe_x)))
        random.shuffle(indices)
        for idx in indices[:num_points]:
            waypoints.append(grid_to_world(safe_x[idx], safe_y[idx]))
    else:
        waypoints = [(0.0, 0.0)]
    return waypoints

# =========================================================================
# 5. MAIN LOOP & STALL RECOVERY
# =========================================================================
print(f"[{robot_id}] Universal engine engaged.")
search_waypoints = generate_safe_waypoints(global_map, num_points=6)
current_target = search_waypoints.pop(0)
current_path = a_star_pathfind((current_x, current_y), current_target)

stuck_counter = 0
last_x, last_y = current_x, current_y

while rosbot.step(timestep) != -1:
    update_odometry()
    
    # --- STALL DETECTION & OBSTACLE ESCAPE BEHAVIOR ---
    distance_moved = math.hypot(current_x - last_x, current_y - last_y)
    if distance_moved < 0.005:
        stuck_counter += 1
    else:
        stuck_counter = 0
    
    last_x, last_y = current_x, current_y
    
    if stuck_counter > 50: 
        print(f"[{robot_id}] STUCK DETECTED: Backing away from obstacle/table...")
        for motor in motors.values(): 
            motor.setVelocity(-3.0)
        rosbot.step(500)
        
        motors["fl"].setVelocity(-3.0)
        motors["rl"].setVelocity(-3.0)
        motors["fr"].setVelocity(3.0)
        motors["rr"].setVelocity(3.0)
        rosbot.step(300)
        
        stuck_counter = 0
        continue
    # -------------------------------------------------
    
    if math.hypot(current_target[0] - current_x, current_target[1] - current_y) < 0.2:
        if len(search_waypoints) > 0:
            current_target = search_waypoints.pop(0)
            current_path = a_star_pathfind((current_x, current_y), current_target)
        else:
            for motor in motors.values(): motor.setVelocity(0.0)
            continue 
    
    if detect_victim():
        for motor in motors.values(): motor.setVelocity(0.0)
        report_victim(current_x, current_y)
    
    left_speed, right_speed = steering_engine.calculate_speeds(
        current_x, current_y, current_heading, current_path
    )
    
    motors["fl"].setVelocity(left_speed)
    motors["rl"].setVelocity(left_speed)
    motors["fr"].setVelocity(right_speed)
    motors["rr"].setVelocity(right_speed)