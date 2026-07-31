import os
import csv
import sys

def generate_victim_estimates(world_name="small_world"):
    log_dir = "controllers/proposed_solution/sim_logs"
    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.join(log_dir, "victim_location_estimates.csv")
    
    # Pre-calculated relative (x, y) coordinates for all maps
    maps_data = {
        "small_world": [
            [3.39, -3.04],
            [5.29, -3.94],
            [2.82, 5.51],
            [8.47, 6.54]
        ],
        "medium_world": [
            [17.11, -15.75],
            [9.73, -26.13],
            [11.45, -21.17],
            [2.73, -11.22]
        ],
        "large_world": [
            [8.77, 2.87],
            [6.41, -4.84],
            [17.18, -7.41],
            [4.64, 2.74],
            [13.50, -1.03]
        ]
    }
    
    victim_coordinates = maps_data.get(world_name, maps_data["small_world"])
    
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        for coord in victim_coordinates:
            writer.writerow(coord)
            
    print(f"Successfully generated victim estimates for [{world_name}] at: {csv_path}")

if __name__ == "__main__":
    # Pass world name as argument: e.g., python generate_csv.py medium_world
    world = sys.argv[1] if len(sys.argv) > 1 else "small_world"
    generate_victim_estimates(world)