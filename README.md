# 2026 IEEE SMCS Search and Rescue (SAR) Competition Submission

## Team Terminal 3

### Team Members

* **Anmol Tiwari**
* **Shrestha Pandey**
* **Kumar Karan Bohidar**

**Faculty Advisor:** Rizwan Ur Rahman

---

# Project Overview

Our solution for the IEEE SMCS Search and Rescue Competition is designed around a modular architecture that separates perception, planning, control, and victim detection. This design improves maintainability, scalability, and ease of testing while remaining fully implemented in **Python** without relying on external web services or compiled custom libraries.

The complete workflow consists of four major subsystems:

1. Video Information Extraction
2. Mission Planning & Pathfinding
3. Multi-Robot Control
4. Victim Detection & Confirmation

---

# System Architecture

## 1. Video Information Extraction

Before the simulation begins, a preprocessing pipeline extracts the environmental layout and initial victim estimates from the aerial flyover video.

The preprocessing script (`map_generator.py`) performs:

* Image smoothing using Gaussian Blur
* Binary thresholding for floor segmentation
* Map generation in the required **600 × 600 RGB** format
* Generation of:

  * `map_estimate.png`
  * `victim_location_estimates.csv`

These files are stored in the simulation log directory and are used by the ground robots during autonomous navigation.

---

## 2. Mission Planning & Pathfinding

Navigation is performed using an **A*** grid-based path planning algorithm.

To improve computational efficiency during simulation:

* The generated map is downsampled from **600 × 600** to **60 × 60**
* Morphological erosion is applied to enlarge obstacle boundaries
* The enlarged obstacles provide a safety margin during navigation and reduce the likelihood of collisions with walls

The resulting path is converted into navigation waypoints for each robot.

---

## 3. Fleet Control & Coordination

### Motion Control

Robot motion is controlled through a proportional steering controller implemented in `steering_pid.py`.

The controller:

* Computes heading error between the robot orientation and target waypoint
* Normalizes heading error within **[-π, π]**
* Applies proportional steering control
* Uses a pivot-and-drive strategy for improved turning accuracy before moving forward

### Multi-Robot Coordination

The search area is divided between the robots to improve coverage efficiency.

Each robot receives an independent search region based on its assigned ID, allowing simultaneous exploration while minimizing overlap.

---

## 4. Victim Detection & Confirmation

Victim identification combines camera-based perception with LiDAR validation.

Detection pipeline:

1. Detect candidate victims using HSV color segmentation
2. Locate the target within the camera frame
3. Verify target distance using the center LiDAR ray
4. Report confirmed victims to the supervisor once the target is within the required detection range

Combining visual and depth information helps reduce false detections.

---

# Repository Structure

```text
controllers/
└── proposed_solution/
    ├── proposed_solution.py
    ├── map_generator.py
    ├── steering_pid.py
    ├── sim_logs/
    │   ├── map_estimate.png
    │   └── victim_location_estimates.csv
    └── ...
```

---

# Requirements

* Python **3.10** or later
* Webots
* NumPy
* OpenCV

---

# Environment Setup

## Option 1 — Conda (Recommended)

```bash
conda env create -f environment.yml
conda activate ieee_sar_env
```

---

## Option 2 — Virtual Environment

### Create a virtual environment

```bash
python -m venv .venv
```

### Activate the environment

**Windows**

```bash
.venv\Scripts\activate
```

**Linux / macOS**

```bash
source .venv/bin/activate
```

### Install dependencies

```bash
pip install -r requirements.txt
```

---

# Running the Project

## Step 1 — Generate Map and Victim Estimates

Run the preprocessing script before launching the simulation.

```bash
python map_generator.py
```

This generates:

* `map_estimate.png`
* `victim_location_estimates.csv`

inside:

```text
controllers/proposed_solution/sim_logs/
```

---

## Step 2 — Start the Simulation

1. Open **Webots**
2. Load the desired world (for example `worlds/large_world.wbt`)
3. Press **Play**

During execution, `proposed_solution.py` automatically:

* Loads the generated map
* Plans navigation paths using A*
* Coordinates the robot fleet
* Detects and confirms victims
* Reports confirmed victims to the supervisor

---

# Technologies Used

* Python 3.10+
* Webots
* OpenCV
* NumPy
* A* Path Planning
* HSV Image Segmentation
* LiDAR-Based Distance Verification
* Proportional Steering Control

---

# Design Highlights

* Modular architecture for perception, planning, and control
* Efficient grid-based A* navigation
* Coordinated multi-robot exploration
* Camera and LiDAR fusion for victim confirmation
* Simple, portable Python implementation suitable for competition deployment

---

# License

Developed as part of the **2026 IEEE SMCS Search and Rescue Competition** submission by **Team Terminal 3**.
