# bumpy_ws

# 🤖 ROS2 Autonomous Mobile Robot (AMR) using Cartographer SLAM

## 📌 Overview

This project implements an **Autonomous Mobile Robot (AMR)** using **ROS2 Humble** on **Ubuntu 22.04**.

The robot is designed for indoor autonomous operation with capabilities such as:

- Real-time LiDAR-based mapping
- SLAM using Google Cartographer
- Map saving
- Autonomous navigation
- Manual teleoperation

The system follows a modular ROS2 architecture with separate packages for robot bringup, sensors, and SLAM.

---

# 🚀 Features

- 🗺️ Real-time SLAM using **Google Cartographer**
- 📡 2D LiDAR-based environment perception
- 💾 Save generated maps (`.yaml` + `.pgm`)
- 🎮 Keyboard teleoperation support
- 🤖 ROS2 modular package architecture
- 📍 Indoor autonomous navigation support
- 🔄 Expandable for future AI and multi-robot applications

---

# 🛠️ Tech Stack

| Component | Technology |
|-----------|------------|
| Operating System | Ubuntu 22.04 |
| Middleware | ROS2 Humble |
| Programming | Python / C++ |
| SLAM | Google Cartographer |
| Visualization | RViz2 |
| Sensor | 2D LiDAR |
| Build System | Colcon |

---

# 📂 Workspace Structure

```
bumpy_ws/
│
├── src/
│   │
│   ├── bumpy_bringup/
│   │    └── launch/
│   │         └── bringup.launch.py
│   │
│   ├── bumpy_sensors/
│   │
│   └── bumpy_slam/
│        │
│        ├── launch/
│        │    ├── slam.launch.py
│        │    ├── cartographer.launch.py
│        │    ├── save_map.launch.py
│        │    └── sllidar_a1_launch.py
│        │
│        ├── config/
│        │    └── cartographer configuration files
│        │
│        ├── maps/
│        │    ├── map.yaml
│        │    └── map.pgm
│        │
│        ├── waypoints/
│        │
│        ├── bumpy_slam/
│        │
│        ├── package.xml
│        ├── setup.py
│        └── setup.cfg
│
├── build/        (ignored)
├── install/      (ignored)
└── log/          (ignored)

```

---

# ⚙️ Installation

## Prerequisites

Install:

- Ubuntu 22.04
- ROS2 Humble
- Colcon build tools

Install required ROS2 packages:

```bash
sudo apt update

sudo apt install ros-humble-cartographer \
ros-humble-cartographer-ros \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-teleop-twist-keyboard
```

---

# 📥 Clone Repository

```bash
git clone https://github.com/rajagopal95/bumpy_ws.git
```

Navigate into workspace:

```bash
cd bumpy_ws
```

---

# 🔨 Build Workspace

Build ROS2 packages:

```bash
colcon build
```

Source workspace:

```bash
source install/setup.bash
```

For every new terminal:

```bash
source ~/bumpy_ws/install/setup.bash
```

---

# ▶️ Running the Robot

## 1️⃣ Start Robot Bringup

Launch robot hardware, TF, motors, and sensors.

```bash
ros2 launch bumpy_bringup bringup.launch.py
```

---

# 🗺️ SLAM Mapping

Start Cartographer SLAM:

```bash
ros2 launch bumpy_slam slam.launch.py
```

This starts:

- LiDAR sensor
- Cartographer SLAM
- TF transformations
- RViz visualization

The robot will generate a map while moving through the environment.

---

# 🎮 Manual Robot Control

Use keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/bumpy7/cmd_vel
```

Robot velocity topic:

```
/bumpy7/cmd_vel
```

Keyboard control:

```
        i

    j   k   l

        ,

```

| Key | Action |
|-----|--------|
| i | Move Forward |
| , | Move Backward |
| j | Rotate Left |
| l | Rotate Right |
| k | Stop |

---

# 💾 Saving Map

After completing SLAM mapping:

```bash
ros2 launch bumpy_slam save_map.launch.py
```

The generated map will be stored inside:

```
src/bumpy_slam/maps/
```

Output files:

```
maps/
 ├── <map_name>.yaml
 └── <map_name>.pgm
```

The `.yaml` file contains map metadata and the `.pgm` file contains the occupancy grid.

---

# 🧭 Autonomous Navigation

After generating a map, the saved map can be used for autonomous navigation.

Navigation workflow:

```
Saved Map
    |
    v
Localization
    |
    v
Path Planning
    |
    v
Velocity Controller
    |
    v
Robot Movement
```

Launch navigation:

```bash
ros2 launch bumpy_navigation navigation.launch.py
```

---

# 🧠 System Workflow

```
              Robot Bringup
                   |
                   v
             Sensor Data
                   |
                   v
              2D LiDAR
                   |
                   v
          Cartographer SLAM
                   |
                   v
          Real-Time Map
                   |
                   v
             Save Map
                   |
                   v
          Localization
                   |
                   v
          Autonomous Navigation

```

---

# 📡 ROS2 Topics

## Velocity Command

```
/bumpy7/cmd_vel
```

Used for:

- Manual control
- Navigation commands


## LiDAR Scan

```
/scan
```

Provides environment information for SLAM.


## TF Frames

Typical TF tree:

```
map
 |
odom
 |
base_link
 |
laser
```

---

# 📸 Results

The system is capable of:

✅ Real-time indoor mapping  
✅ LiDAR-based obstacle detection  
✅ Map generation and saving  
✅ Manual robot operation  
✅ Autonomous navigation preparation  


(Add RViz screenshots here)

---

# 🔮 Future Improvements

- 🔗 Sensor fusion (LiDAR + IMU + Camera)
- 🧠 AI-based object detection
- 🚧 Dynamic obstacle avoidance
- 🤝 Multi-robot coordination
- ☁️ Remote monitoring dashboard
- 📍 Improved localization accuracy

---

# 👨‍💻 Author

**Raja**

---

# 📜 License

This project is developed for educational and research purposes.
