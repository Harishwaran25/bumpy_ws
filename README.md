# 🤖 bumpy_ws — ROS2 Autonomous Mobile Robot Platform

A ROS2-based Autonomous Mobile Robot (AMR) platform supporting **LiDAR-based SLAM, sensor fusion (EKF), autonomous navigation, teleoperation, camera/gas sensing, and multi-robot formation control**.

The workspace is developed and tested using **ROS2 Humble on Ubuntu 22.04**.

---

# 📌 Overview

`bumpy_ws` is a modular ROS2 workspace built for a differential-drive autonomous mobile robot (bumpy7) and multi-robot experimentation.

The system uses a **YDLidar 2D LiDAR** for environment perception, an **EKF-based sensor fusion pipeline**, and supports SLAM, autonomous navigation, and multi-robot formation control.

The workspace contains:

- Robot bringup
- Robot description (URDF/xacro) + EKF sensor fusion
- Low-level firmware (motor control, teleop, tick publishing)
- Sensor drivers (camera, IMU, gas sensor, OLED, web dashboard, leader election)
- SLAM (Cartographer)
- Autonomous navigation (Nav2)
- Multi-robot formation control
- YDLidar ROS2 driver + SDK

The architecture is designed to be extendable for **multi-robot coordination, swarm robotics, warehouse automation, and autonomous indoor navigation**.

---

# 🚀 Features

### 🤖 Autonomous Robot

- ROS2-based robot control (`bumpy_bringup`)
- Differential-drive URDF/xacro description (`bumpy_description`)
- EKF-based sensor fusion (`ekf.yaml`)
- TF management
- Velocity control via `diff_tf.py` / `motor_control.py`

### 🗺️ SLAM & Mapping

- Google Cartographer SLAM (`bumpy_slam`)
- Real-time 2D map generation
- Map saving (`save_map.launch.py`)
- Multiple saved maps (`amr_map`, `maze`, `room_map`, `SREC_COE_NEW`)
- Saved waypoints per map (JSON)

### 🧭 Navigation

- Nav2-based autonomous navigation (`bumpy_navigation`)
- Configurable params (`nav2_params.yaml`)

### 📡 Sensors

- Camera + camera TF publisher
- IMU node
- Gas sensor node
- OLED status display
- Robot status monitor
- Web server dashboard (Flask-based)
- Leader election node (multi-robot)

### 🎮 Firmware & Teleoperation

- Differential-drive tick publisher + motor control
- Xbox controller teleop config
- Keyboard teleoperation

### 🤝 Multi-Robot Coordination

The `formation_control` package contains:

- 2-robot formation control
- 3-robot formation control (two implementations)
- Obstacle handling for multi-robot formations

### 📟 LiDAR

- Custom `ydlidar_ros2_driver` package (C++ driver, RViz configs)
- Vendored `YDLidar-SDK` for hardware communication

---

# 🛠️ Technology Stack

| Component | Technology |
|---|---|
| Operating System | Ubuntu 22.04 |
| Middleware | ROS2 Humble |
| Programming | Python / C++ |
| SLAM | Google Cartographer |
| Navigation | ROS2 Navigation Stack (Nav2) |
| Sensor Fusion | robot_localization (EKF) |
| Visualization | RViz2 |
| Environment Perception | YDLidar 2D LiDAR |
| Build System | Colcon |
| Multi-Robot Control | Python |
| Robot Communication | ROS2 Topics / TF |

---

# 📂 Project Structure

```text
bumpy_ws/
│
├── src/
│   │
│   ├── bumpy_bringup/              # Top-level robot bringup
│   │   ├── bumpy_bringup/
│   │   ├── launch/
│   │   │   ├── bringup.launch.py
│   │   │   └── minimal.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── test/
│   │
│   ├── bumpy_description/          # URDF/xacro + EKF config
│   │   ├── bumpy_description/
│   │   ├── config/
│   │   │   └── ekf.yaml
│   │   ├── launch/
│   │   │   └── robot.launch.py
│   │   ├── urdf/
│   │   │   ├── bumpy.xacro
│   │   │   ├── bumpy7.xacro
│   │   │   └── robot.xacro
│   │   └── test/
│   │
│   ├── bumpy_firmware/             # Low-level motor control & teleop
│   │   ├── bumpy_firmware/
│   │   │   ├── diff_tf.py
│   │   │   ├── motor_control.py
│   │   │   ├── teleop.py
│   │   │   └── tick_pub.py
│   │   ├── launch/
│   │   │   └── motor_control.launch.py
│   │   └── test/
│   │
│   ├── bumpy_navigation/           # Nav2 autonomous navigation
│   │   ├── bumpy_navigation/
│   │   ├── launch/
│   │   │   └── navigation.launch.py
│   │   ├── params/
│   │   │   └── nav2_params.yaml
│   │   └── test/
│   │
│   ├── bumpy_sensors/               # Sensor drivers & dashboard
│   │   ├── bumpy_sensors/
│   │   │   ├── camera.py
│   │   │   ├── camera_tf_pub.py
│   │   │   ├── gas_sensor_node.py
│   │   │   ├── imu.py
│   │   │   ├── leader_ele.py
│   │   │   ├── oled.py
│   │   │   ├── robot_status.py
│   │   │   └── web_server_node.py
│   │   └── test/
│   │
│   ├── bumpy_slam/                 # Cartographer SLAM + maps
│   │   ├── bumpy_slam/
│   │   ├── config/
│   │   │   ├── slam.lua
│   │   │   └── slam_params.yaml
│   │   ├── launch/
│   │   │   ├── slam.launch.py
│   │   │   ├── cartographer.launch.py
│   │   │   ├── save_map.launch.py
│   │   │   └── sllidar_a1_launch.py
│   │   ├── maps/
│   │   │   ├── amr_map.pgm / .yaml
│   │   │   ├── maze.pgm / .yaml
│   │   │   ├── room_map.pgm / .yaml
│   │   │   └── SREC_COE_NEW.pgm / .yaml
│   │   ├── waypoints/
│   │   │   ├── bumpy_map_waypoints.json
│   │   │   ├── room_map_waypoints.json
│   │   │   └── SREC_COE_NEW_waypoints.json
│   │   └── test/
│   │
│   ├── formation_control/          # Multi-robot formation control
│   │   ├── config/
│   │   │   └── formation_config.yaml
│   │   ├── formation_control/
│   │   │   ├── 2robot.py
│   │   │   ├── 3robot.py
│   │   │   ├── 3robot2.py
│   │   │   └── obstacle.py
│   │   ├── launch/
│   │   │   └── formation.launch.py
│   │   └── test/
│   │
│   ├── ydlidar_ros2_driver/        # YDLidar ROS2 driver (C++)
│   │   ├── config/
│   │   │   └── ydlidar.rviz
│   │   ├── launch/
│   │   │   ├── ydlidar_launch.py
│   │   │   └── ydlidar_launch_view.py
│   │   ├── params/
│   │   │   └── *.yaml   (per LiDAR model)
│   │   └── src/
│   │       ├── ydlidar_ros2_driver_client.cpp
│   │       └── ydlidar_ros2_driver_node.cpp
│   │
│   └── YDLidar-SDK/                # Vendored LiDAR SDK
│       ├── core/
│       ├── src/
│       ├── python/
│       └── doc/
│
├── build/        # Ignored
├── install/      # Ignored
└── log/          # Ignored
```

> `__pycache__/`, `.pyc`, and `.bak`/`.backup` files should be excluded from Git via `.gitignore`.

---

# ⚙️ Installation

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- Colcon
- RViz2
- Cartographer
- Navigation2
- robot_localization (EKF)

Install required ROS2 packages:

```bash
sudo apt update

sudo apt install \
ros-humble-cartographer \
ros-humble-cartographer-ros \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-robot-localization \
ros-humble-teleop-twist-keyboard \
ros-humble-rviz2
```

---

# 📥 Clone the Repository

```bash
git clone https://github.com/rajagopal95/bumpy_ws.git
cd bumpy_ws
```

---

# 🔨 Build the Workspace

```bash
colcon build
source install/setup.bash
```

For every new terminal:

```bash
source ~/bumpy_ws/install/setup.bash
```

---

# 🤖 Robot Bringup

```bash
ros2 launch bumpy_bringup bringup.launch.py
```

Bringup starts:

- Robot description (URDF/TF)
- EKF sensor fusion
- Motor control / firmware nodes
- Sensor nodes

For a lightweight bringup (development/testing):

```bash
ros2 launch bumpy_bringup minimal.launch.py
```

---

# 📡 LiDAR

The robot uses a **YDLidar 2D LiDAR** for environment perception.

Launch the driver:

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

Or via the SLAM package's wrapper:

```text
src/bumpy_slam/launch/sllidar_a1_launch.py
```

---

# 🧮 Sensor Fusion (EKF)

Odometry and IMU data are fused using `robot_localization`, configured at:

```text
src/bumpy_description/config/ekf.yaml
```

This publishes a filtered `odom → base_link` transform used by SLAM and Nav2.

---

# 🗺️ SLAM

The project uses **Google Cartographer** for real-time SLAM.

```bash
ros2 launch bumpy_slam slam.launch.py
```

Cartographer parameters:

```text
src/bumpy_slam/config/slam.lua
src/bumpy_slam/config/slam_params.yaml
```

---

# 🎮 Manual Teleoperation

Keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

## Keyboard Controls

```text
        i
    j   k   l
        ,
```

| Key | Function |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate Left |
| `l` | Rotate Right |
| `k` | Stop |

---

# 💾 Saving a Map

```bash
ros2 launch bumpy_slam save_map.launch.py
```

Saved maps are stored in:

```text
src/bumpy_slam/maps/
```

Each map includes a `.yaml` (metadata) and `.pgm` (occupancy grid image).

Saved waypoints for each map live in:

```text
src/bumpy_slam/waypoints/
```

---

# 🧭 Autonomous Navigation

```bash
ros2 launch bumpy_navigation navigation.launch.py
```

Navigation pipeline:

```text
                Saved Map
                    │
                    ▼
              Localization
                    │
                    ▼
             Global Planner
                    │
                    ▼
              Local Planner
                    │
                    ▼
              Controller
                    │
                    ▼
                 /cmd_vel
                    │
                    ▼
                  Robot
```

Nav2 parameters:

```text
src/bumpy_navigation/params/nav2_params.yaml
```

---

# 📷 Sensors & Dashboard

The `bumpy_sensors` package provides:

- `camera.py` / `camera_tf_pub.py` — camera driver + TF publishing
- `imu.py` — IMU data publishing
- `gas_sensor_node.py` — gas sensor readings
- `oled.py` — OLED status display
- `robot_status.py` — robot health/status monitoring
- `leader_ele.py` — leader election for multi-robot setups
- `web_server_node.py` — Flask-based web dashboard for monitoring

---

# 🤝 Multi-Robot Formation Control

```text
src/formation_control/
```

Implementations:

```text
formation_control/
├── 2robot.py     # Two-robot formation
├── 3robot.py     # Three-robot formation (v1)
├── 3robot2.py    # Three-robot formation (v2)
└── obstacle.py   # Obstacle handling for formations
```

## 👥 Two-Robot Formation

```text
        Robot 1
           │
           ▼
      Formation Logic
           │
           ▼
        Robot 2
```

## 👥👥 Three-Robot Formation

```text
             Robot 1
                ▲
               / \
              /   \
             /     \
        Robot 2 ─── Robot 3
```

## 🚧 Obstacle Handling

```text
              Obstacle
                 ███
                 ███
                  │
                  ▼

        Robot 1
           \
            \
        Robot 2 -----> Formation Motion
            /
           /
        Robot 3
```

---

# 🔄 Multi-Robot System Architecture

```text
                 Environment
                      │
                      ▼
               LiDAR / Sensors
                      │
                      ▼
                Robot Perception
                      │
                      ▼
             Formation Controller
                      │
          ┌───────────┼───────────┐
          │           │           │
          ▼           ▼           ▼
       Robot 1     Robot 2     Robot 3
          │           │           │
          └───────────┼───────────┘
                      │
                      ▼
               Coordinated Motion
```

---

# 🧠 Overall System Workflow

```text
                         START
                           │
                           ▼
                    Robot Bringup
                           │
                           ▼
                 Sensor + EKF Startup
                           │
                           ▼
                    YDLidar 2D LiDAR
                           │
                           ▼
                    Cartographer
                           │
                           ▼
                    Real-Time SLAM
                           │
                           ▼
                       Save Map
                           │
                           ▼
                 Nav2 Localization
                           │
                           ▼
                 Autonomous Navigation
                           │
                           ▼
                  Multi-Robot Control
                           │
                           ▼
                Formation + Obstacle
                     Handling
                           │
                           ▼
                          END
```

---

# 📊 ROS2 Communication

Important interfaces include:

### Velocity Command

```text
/cmd_vel
```

### LiDAR

```text
/scan
```

### Odometry / TF

```text
/odom
odom → base_link (EKF-fused)
```

To inspect available topics:

```bash
ros2 topic list
```

To inspect the TF tree:

```bash
ros2 run tf2_tools view_frames
```

---

# 📟 YDLidar Driver & SDK

The `ydlidar_ros2_driver` package (C++) wraps the vendored `YDLidar-SDK` to provide ROS2 LaserScan output.

Build the SDK (if not already built):

```bash
cd src/YDLidar-SDK
mkdir -p build && cd build
cmake ..
make
sudo make install
```

Then build the ROS2 driver as part of the workspace via `colcon build`.

---

# 📁 Recommended Git Ignore

```gitignore
# ROS2
build/
install/
log/

# Python
__pycache__/
*.pyc

# Backups
*.bak
*.bak2
*.backup
*.backup.*

# YDLidar-SDK build artifacts
src/YDLidar-SDK/build/

# IDE
.vscode/
.idea/

# OS
.DS_Store
```

---

# 📸 Results

The system is designed to demonstrate:

- ✅ EKF-based sensor fusion
- ✅ Real-time LiDAR mapping
- ✅ Cartographer SLAM
- ✅ Saved occupancy-grid maps + waypoints
- ✅ Nav2 autonomous navigation
- ✅ Manual keyboard robot control
- ✅ Sensor dashboard (camera, IMU, gas, OLED, web)
- ✅ Multi-robot formation control
- ✅ Obstacle-aware multi-robot development

Add project screenshots here:

```text
docs/
├── mapping.png
├── navigation.png
├── rviz.png
└── formation_control.png
```

Example:

```markdown
![SLAM Mapping](docs/mapping.png)
```

---

# 🔮 Future Improvements

### Navigation

- Dynamic obstacle avoidance
- Improved local planning
- Adaptive planner selection
- Improved localization

### Perception

- LiDAR + IMU fusion refinement
- LiDAR + Camera fusion
- AI-based object detection (YOLOv8)
- Dynamic obstacle tracking

### Multi-Robot

- Dynamic formation switching
- Leader-follower control
- Decentralized coordination
- Collision avoidance
- Multi-robot task allocation
- Swarm intelligence

### Remote Operation

- Web-based robot monitoring
- Multi-robot dashboard
- Remote teleoperation
- Real-time status monitoring

---

# 🎯 Potential Applications

- 🏭 Warehouse automation
- 📦 Material transportation
- 🏥 Hospital logistics
- 🏢 Indoor service robots
- 🚨 Surveillance
- 🔍 Inspection
- 🤝 Multi-robot exploration
- 🐝 Swarm robotics research

---

# 👨‍💻 Author

**N N Rajagopal**

ROS2 | Autonomous Mobile Robots | SLAM | Navigation | Multi-Robot Systems

Portfolio: rajagopal95.github.io/Portfolio

---

# 📜 License

This project is developed for **educational, development, and research purposes**.
