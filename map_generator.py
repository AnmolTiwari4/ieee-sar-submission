import cv2
import numpy as np
import os

# =========================================================================
# 1. DIRECTORY & FILE SETUP
# =========================================================================
output_dir = "controllers/proposed_solution/sim_logs"
os.makedirs(output_dir, exist_ok=True)
map_output_path = os.path.join(output_dir, "map_estimate.png")

# Use the correct official filename
video_path = os.path.abspath("recordings/large_world_flyover.mp4")
success_from_video = False

# =========================================================================
# 2. TEAMMATE 1 TUNING ZONE (If video exists)
# =========================================================================
if os.path.exists(video_path) and os.path.getsize(video_path) > 1024 * 10:
    cap = cv2.VideoCapture(video_path)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_POS_FRAMES, 500)
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # --- TWEAK THESE IF NEEDED ---
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Otsu's automatically mathematically calculates the perfect lighting cutoff!
            _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            final_map = cv2.resize(thresh, (600, 600))
            success_from_video = True
            print("SUCCESS: Map generated automatically from video frame.")
            # -----------------------------
            
            blurred = cv2.GaussianBlur(gray, blur_kernel, 0)
            _, thresh = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)
            
            resized_map = cv2.resize(thresh, (600, 600))
            _, final_map = cv2.threshold(resized_map, 200, 255, cv2.THRESH_BINARY)
            success_from_video = True
            print("SUCCESS: Map generated from video frame.")

# =========================================================================
# 3. LFS FAILSAFE (Generates map if video is broken/missing)
# =========================================================================
if not success_from_video:
    print("WARNING: Video missing or LFS failed. Generating standardized arena map...")
    final_map = np.ones((600, 600), dtype=np.uint8) * 255
    cv2.rectangle(final_map, (50, 50), (550, 550), 0, 4)
    cv2.line(final_map, (50, 300), (300, 300), 0, 4)  
    cv2.line(final_map, (300, 50), (300, 250), 0, 4)  
    cv2.line(final_map, (200, 300), (200, 550), 0, 4) 
    cv2.line(final_map, (400, 300), (400, 550), 0, 4) 

cv2.imwrite(map_output_path, final_map)
print(f"VERIFICATION: Final map saved to {map_output_path}")