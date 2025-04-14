<!-- Logo & Title -->
<p align="center">
  <img src="logo.png" alt="Cell Sorting Simulation Platform Logo" width="120" />
</p>

# Cell Sorting Simulation Platform

[Project Page](#) · [Visualization Demo](#) · [Documentation](#) · [Tutorial Video](#)

---

## Introduction

This project includes the necessary code and simulation environment for **bimanual haptic feedback control**, **RL-based autonomous control**, and **shared control navigation for cell sorting**. To facilitate the demonstration of the visual rendering demo for visitors, the repository includes a [`Visualization_rendering_video.zip`](./Visualization_rendering_video.zip) which contains robot images at various poses and depths. Following the tutorial, you can replicate the cell sorting demo with ease.

---

## Usage

### Preparation

- **Hardware Requirements**:
  - Two computers with **ROS1** (ROS2 is not supported) for bimanual haptic feedback control.
  - Only one computer (the main computer) needs to have **Isaac Sim** installed to run the RL navigation code.
- **Software Requirements**:
  - ROS1 installed on both machines.
  - Isaac Sim installed on the main computer.
  - Ensure that extraction paths do **not** contain any non-English (e.g., Chinese) characters.

---

### Bimanual Haptic Feedback Control

1. **Download** the `Bimanual_haptic_feedback_control_code` package.
2. **Deploy** the following workspaces on two separate computers configured with Geomagic Touch devices:
   - `Left_hand_geomagic_touch_control`
   - `Right_hand_geomagic_touch_control`
3. **Compile** the code in each workspace (C++ ROS workspace) and complete the basic setup.

---

### RL Navigation Setup

1. **Download** the `RL_navigation_code` and `3DModel` packages.
2. **Extract** the files to directories without Chinese characters.
3. **Placement**: Place the RL navigation code in the same directory as Isaac Sim's built-in standalone example code. This ensures that the code has access to the necessary compilation environment.
4. **Update** file paths in the code where necessary:
   - **DQN_env**: Drives the robot for shared control.
   - **deploy_best_model**: Drives the robot for full autonomous control.
   - **best_model_750.pth**: Contains the RL-trained network parameters.
   - **smoothed_path.csv**: Stores the optimal path calculated by the A* algorithm.

---

### Simulation Environment

1. **Download** all files starting with `Sim_env` and extract them.
2. **Start ROS** (e.g., run `roscore`) on your laptop.
3. **Load USD Files** in NVIDIA Omniverse via the ROS1 bridge and update all referenced USD file paths to avoid errors.

---

### Running the Demo

1. **Run** the Geomagic Touch code on the two computers.
2. **Launch** the Isaac Sim simulation environment on the main computer.
3. **Execute** the RL navigation code to run the cell sorting demo.

---

## File Structure Explanation

```tree
.
├── Bimanual_haptic_feedback_control_code.zip
│   └── Contains the code for controlling Geomagic Touch devices.
│       - Left_hand_geomagic_touch_control (deploy on machine 1)
│       - Right_hand_geomagic_touch_control (deploy on machine 2)
├── RL_navigation_code.zip
│   ├── DQN_env               # Used for shared control in simulation
│   ├── deploy_best_model     # Used for full autonomous control
│   ├── best_model_750.pth    # RL-trained network parameters
│   └── smoothed_path.csv     # Optimal path from A* algorithm
├── 3DModel.zip
│   └── Contains USD files for mechanical structures (converted from STL files)
├── Rendered Images Raw Data.zip
│   └── Contains rendered robot images with various poses and depths for demo
└── Sim_env_* 
    └── Files for setting up the simulation environment in Isaac Sim
