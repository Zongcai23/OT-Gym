# 📝 OT-Gym: Optical Tweezers Gym

## ✍️ Authors & Affiliations  
**Zongcai Tan**, Imperial College London  
**Dandan Zhang**, Imperial College London  

## 🔗 Project Links  
- **Website:** [ot-gym.github.io](https://your-project-website.example.com)  
- **Paper:** [arXiv:XXXX.XXXXX](https://arxiv.org/abs/XXXX.XXXXX)  

## 🛠️ Status & Requirements  
![IsaacSim 3.10](docs/badges/isaacsim-3.10-blue.svg)  
![Isaac Lab](docs/badges/isaaclab-python3.10-linux64-green.svg)  
![Platform: Linux-64](docs/badges/platform-linux64.svg)  
![ROS1](docs/badges/ros1-yellow.svg)  
![License: Apache-2.0](docs/badges/license-apache2.0.svg)  

## 📖 Overview  
![Overview Diagram](docs/overview.png)  
OT-Gym is a physics-based optical micro-robot simulation framework built on NVIDIA Isaac Sim to train reinforcement-learning algorithms for micro-manipulation tasks. This repo provides code and simulation environments for:  
- **Bimanual haptic-feedback control**  
- **RL-based autonomous control**  
- **Shared-control navigation for cell sorting**  
using optical-tweezer-actuated microrobots.  

## ✨ Features  

### 🤖 Shared OT Micromanipulation Process  
![Shared OT Micromanipulation GIF](gifs/shared_ot_process.gif)  
Users first indirectly grasp target cells using optical robots, then transport the cells to designated locations before releasing them.  

### 📈 Effectiveness Validation Experiment  
![Shared Control Validation GIF](gifs/shared_control_validation.gif)  
Manual control is safe yet inefficient, autonomous control is efficient but collision-prone, and shared control balances efficiency with safety.  

## 📋 Requirements  

### 🖥️ Hardware  
- Two **Geomagic Touch** haptic devices for teleoperation  
- Two computers with **ROS 1** (ROS 2 _not_ supported)  
- One “main” computer with NVIDIA Isaac Sim installed  

### 💾 Software  
- **ROS 1** on both machines  
- **Isaac Sim** (latest) on the main machine  
- Extraction paths must contain only ASCII characters  
- All devices on the same LAN  

## 📚 User Guide  

### 1. Basic Setup  
OT-Gym builds on NVIDIA Isaac Sim & Isaac Lab. See the [Isaac Lab install guide](https://docs.omniverse.nvidia.com/isaac/lab/latest/install.html) and [Isaac Sim docs](https://docs.omniverse.nvidia.com/isaac/reactivesim/latest/install.html).  

### 2. Bimanual Haptic-Feedback Control  
```bash
# Download & unzip
unzip Bimanual_haptic_feedback_control_code.zip

# Machine 1 (Left hand)
cd Left_hand_geomagic_touch_control
catkin_make

# Machine 2 (Right hand)
cd Right_hand_geomagic_touch_control
catkin_make

# On each machine
roscore                # if not already running
roslaunch left_touch_control demo.launch  # or right_touch_control
```

### 3. RL Navigation Setup  
```bash
# Download & unzip RL navigation & 3D models
unzip RL_navigation_code.zip
unzip 3DModel.zip

# Ensure ASCII-only paths
# Place RL code alongside Isaac Sim examples
mv RL_navigation_code path/to/IsaacSim/examples/

# Update paths in code if needed:
# - DQN_env → shared control
# - deploy_best_model → autonomous control
# - best_model_750.pth → RL weights
# - smoothed_path.csv → A* path
```

### 4. Simulation Environment  
```bash
# Download & unzip Sim environment
unzip Sim_env_*.zip

# On main computer:
roscore
```  
- Load USD files via the ROS1 bridge in Omniverse  
- Update USD file paths to avoid errors  

### 5. Running the Demo  
```bash
# Start Geomagic Touch on both machines
roslaunch left_touch_control demo.launch
roslaunch right_touch_control demo.launch

# Launch Isaac Sim on main machine
./launchIsaacSim.sh

# Run RL demo
rosrun rl_navigation deploy_best_model.py
```

## 📂 File Structure  
```plaintext
.
├── docs/
│   ├── badges/                     # badge SVGs
│   └── overview.png                # overview diagram
├── gifs/
│   ├── shared_ot_process.gif
│   └── shared_control_validation.gif
├── Bimanual_haptic_feedback_control_code.zip
├── RL_navigation_code.zip
├── 3DModel.zip
├── Rendered_Images_Raw_Data.zip
└── Sim_env_*.zip
```
