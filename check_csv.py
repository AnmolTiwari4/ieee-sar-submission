import os

csv_path = "controllers/proposed_solution/sim_logs/victim_location_estimates.csv"

if os.path.exists(csv_path):
    with open(csv_path, "r") as f:
        lines = f.readlines()
        print(f"Total lines found: {len(lines)}")
        for i, line in enumerate(lines[:5]): # Print first 5 lines
            print(f"Line {i+1}: {line.strip()}")
else:
    print("CSV file not found at the expected path!")