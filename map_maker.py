import cv2
import numpy as np
import os

# --- RED ALERT FIX: Ensure the strict hidden folders exist ---
output_dir = "controllers/proposed_solution/sim_logs"
os.makedirs(output_dir, exist_ok=True)

# 1. Load the Drone Video
video_path = "recordings/large_world_flyover.mp4"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("CRITICAL ERROR: Cannot open video. Did Git LFS download the mp4 correctly?")
    exit()

# 2. Extract a High-Altitude Frame
# We don't want to process the whole video. We just want one frame from the middle 
# where the drone is high up and can see the maze.
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
middle_frame = total_frames // 2

# Fast-forward the video to the middle frame
cap.set(cv2.CAP_PROP_POS_FRAMES, middle_frame)
success, frame = cap.read()

if not success:
    print("CRITICAL ERROR: Could not read frame.")
    exit()

print("Mission Control: Video loaded and frame extracted successfully.")

# Save the raw frame just so you can look at it and verify the drone is high enough
cv2.imwrite("test_raw_frame.png", frame)
print("Saved test_raw_frame.png to your folder. Open it and check the view.")

cap.release()