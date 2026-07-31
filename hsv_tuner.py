import cv2
import numpy as np
import os

def get_hsv_bounds(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        frame, hsv_frame = param
        
        # Extract the exact pixel you clicked
        hsv_pixel = hsv_frame[y, x]
        h, s, v = hsv_pixel
        
        # Mathematically calculate the upper and lower bounds automatically
        lower_h = max(0, h - 10)
        upper_h = min(179, h + 10)
        lower_s = max(0, s - 50)
        upper_s = min(255, s + 50)
        lower_v = max(0, v - 50)
        upper_v = min(255, v + 50)
        
        print("\n--- PHASE 1 COMPLETE ---")
        print(f"Clicked Pixel (X:{x}, Y:{y})")
        print(f"Exact OpenCV HSV of Victim: [{h}, {s}, {v}]")
        print("\nCopy and paste this directly into your logic file:")
        print(f"lower_color = np.array([{lower_h}, {lower_s}, {lower_v}])")
        print(f"upper_color = np.array([{upper_h}, {upper_s}, {upper_v}])")
        print("------------------------\n")

# Bulletproof path resolution
script_dir = os.path.dirname(os.path.abspath(__file__))
video_path = os.path.join(script_dir, "recordings", "large_world_flyover.mp4")

# LFS Check & Fallback
use_simulated_patch = False
if os.path.exists(video_path):
    if os.path.getsize(video_path) < 1024 * 10:  # Less than 10KB means it's a Git LFS text pointer
        print(f"WARNING: {video_path} is only {os.path.getsize(video_path)} bytes.")
        print("Git LFS failed to download the real video. Using fallback simulated color patch...")
        use_simulated_patch = True
else:
    print("WARNING: Video not found in recordings/. Using fallback simulated color patch...")
    use_simulated_patch = True

if not use_simulated_patch:
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 150)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        use_simulated_patch = True

# Failsafe: Generate a simulated color patch if the video fails entirely
if use_simulated_patch:
    frame = np.zeros((400, 400, 3), dtype=np.uint8)
    # BGR color for Webots Hazmat Orange/Yellow
    cv2.rectangle(frame, (100, 100), (300, 300), (0, 165, 255), -1)
    cv2.putText(frame, "Simulated Victim Patch", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
print("\nClick directly on the victim (or simulated patch) to extract HSV bounds.")
print("Press 'q' or Esc to close the window.")

cv2.imshow("Phase 1: Automatic HSV Tuner", frame)
cv2.setMouseCallback("Phase 1: Automatic HSV Tuner", get_hsv_bounds, param=(frame, hsv_frame))

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break
cv2.destroyAllWindows()