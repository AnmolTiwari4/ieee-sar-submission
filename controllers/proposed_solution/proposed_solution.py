from controller import Robot
import json
import cv2
import numpy as np
import math
import heapq

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
current_x = -0.375 if robot_id == "robot1" else -0.375
current_y = 0.375 if robot_id == "robot1" else 0.0
current_heading = 0.0
last_encoder_values = {"fl": 0.0, "fr": 0.0}

# Constants for ROSbot
WHEEL_RADIUS = 0.0425
WHEEL_BASE = 0.192

# =========================================================================
# 4. ANMOL'S LOGIC ENGINES
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
    """Uses OpenCV HSV masking and LiDAR depth to confirm a victim."""
    raw_img = camera.getImage()
    if not raw_img: 
        return False
    
    img_array = np.frombuffer(raw_img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    frame_bgr = img_array[:, :, :3]
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    
    # TODO: Tune HSV values to match the victim in Webots
    lower_color = np.array([5, 100, 100])
    upper_color = np.array([15, 255, 255])
    
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)
    
    if cv2.countNonZero(mask) > 1500: 
        depth_data = lidar.getRangeImage()
        center_depth = depth_data[len(depth_data) // 2]
        
        if center_depth < 1.0:
            return True
            
    return False

def a_star_pathfind(start_node, target_node):
    """Placeholder A* Algorithm for Anmol's grid navigation."""
    pass

# =========================================================================
# 5. MAIN AUTONOMY LOOP
# =========================================================================
print(f"[{robot_id}] Systems green. Engaging autonomous search.")

while rosbot.step(timestep) != -1:
    
    update_odometry()
    
    if detect_victim():
        for motor in motors.values(): 
            motor.setVelocity(0.0)
        report_victim(current_x, current_y)
    
    left_speed = 2.0  
    right_speed = 2.0
    
    motors["fl"].setVelocity(left_speed)
    motors["rl"].setVelocity(left_speed)
    motors["fr"].setVelocity(right_speed)
    motors["rr"].setVelocity(right_speed)