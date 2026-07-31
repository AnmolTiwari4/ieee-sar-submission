import cv2
import numpy as np
import csv
import argparse

def process_flyover(video_path):
    cap = cv2.VideoCapture(video_path)
    estimated_victims = []
    
    # Initialize ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    origin_center = None
    scale_factor = None  # pixels per meter

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        # 1. Detect ArUco markers to establish the coordinate frame
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None and 0 in ids:
            # Find index of Marker 0
            idx = np.where(ids == 0)[0][0]
            c = corners[idx][0]
            # Center of Marker 0 represents (0,0) origin
            origin_center = np.mean(c, axis=0)
            
            # If Marker 2 is also visible, compute scale factor (distance between marker 0 and 2 is known)
            if 2 in ids:
                idx2 = np.where(ids == 2)[0][0]
                c2 = np.mean(corners[idx2][0], axis=0)
                pixel_distance = np.linalg.norm(c2 - origin_center)
                # Distance from Marker 0 center to Marker 2 center is 0.5 meters (example layout math)
                scale_factor = pixel_distance / 0.5 

        # 2. Process frames to find victims (using color thresholding/contours)
        # Once you find a victim contour at pixel (u, v) and have your origin/scale:
        if origin_center is not None and scale_factor is not None:
            # Example transformation once a victim pixel (u, v) is isolated:
            # x_meters = (u - origin_center[0]) / scale_factor
            # y_meters = (v - origin_center[1]) / scale_factor
            # estimated_victims.append([round(x_meters, 2), round(y_meters, 2)])
            pass

    cap.release()
    
    # Ensure duplicates are filtered out and format cleanly
    unique_victims = [list(x) for x in set(tuple(x) for x in estimated_victims)]
    
    # Save directly to the required grading path
    csv_path = "controllers/proposed_solution/sim_logs/victim_location_estimates.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        for coord in unique_victims:
            writer.writerow(coord)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, help="Path to flyover mp4 video")
    args = parser.parse_args()
    process_flyover(args.file)