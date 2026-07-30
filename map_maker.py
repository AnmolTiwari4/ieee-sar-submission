import cv2
import numpy as np
import os

# --- RED ALERT FIX: Ensure the strict hidden folders exist ---
output_dir = "controllers/proposed_solution/sim_logs"
os.makedirs(output_dir, exist_ok=True)

# 1. Open the video using the absolute path
absolute_path = os.path.abspath("recordings/large_world_flyover.mp4")
cap = cv2.VideoCapture(absolute_path)

if not cap.isOpened():
    print("CRITICAL ERROR: Cannot open video.")
    exit()

# 2. Jump straight to our winning takeoff frame (Frame 90)
winning_frame_number = 500
cap.set(cv2.CAP_PROP_POS_FRAMES, winning_frame_number)
success, frame = cap.read()

if not success:
    print("CRITICAL ERROR: Could not read winning frame.")
    exit()

print(f"Mission Control: Locked onto takeoff frame {winning_frame_number}.")

# 3. Process the Map for the Marking Server
# Convert to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Run Canny Edge Detection to outline the walls
edges = cv2.Canny(gray, 50, 150)

# Invert colors so walls are white and background is black temporarily
inverted_edges = cv2.bitwise_not(edges)

# Resize strictly to the judges' required 600x600 dimensions
resized_map = cv2.resize(inverted_edges, (600, 600))

# --- STRICT COLOR ENFORCEMENT ---
# The rules state: Black (0,0,0) for walls, White (255,255,255) for free space. No gray pixels allowed!
# We use a threshold to force every single pixel to be pure 0 or pure 255.
_, final_map = cv2.threshold(resized_map, 200, 255, cv2.THRESH_BINARY)

# 4. Save the map directly to the strict required path
map_output_path = os.path.join(output_dir, "map_estimate.png")
cv2.imwrite(map_output_path, final_map)

print(f"SUCCESS: Strictly formatted 600x600 map saved to {map_output_path}!")
# Verify dimensions before finishing

height, width = final_map.shape
print(f"VERIFICATION: Final map dimensions are {width}x{height}")

cap.release()