# 🤖 bumpy_ws — ROS2 Autonomous Mobile Robot Platform

A ROS2-based Autonomous Mobile Robot (AMR) platform supporting **LiDAR-based SLAM, mapping, autonomous navigation, RViz visualization, teleoperation, and multi-robot formation control**.

The workspace is developed and tested using **ROS2 Humble on Ubuntu 22.04**.

---

# 📌 Overview

`bumpy_ws` is a modular ROS2 workspace developed for an autonomous mobile robot platform.

The system uses a **2D LiDAR** for environment perception and **Google Cartographer** for SLAM and mapping.

The workspace also contains:

- Robot bringup
- LiDAR sensor integration
- Cartographer SLAM
- Map saving
- Autonomous navigation
- Keyboard teleoperation
- RViz configurations
- TF visualization
- Development utilities
- Multi-robot formation control
- Obstacle handling for multi-robot systems

The architecture is designed to be extendable for future applications such as **multi-robot coordination, swarm robotics, warehouse automation, and autonomous indoor navigation**.

---

# 🚀 Features

### 🤖 Autonomous Robot

- ROS2-based robot control
- LiDAR-based environment perception
- Robot bringup
- TF management
- Velocity control
- Autonomous navigation

### 🗺️ SLAM & Mapping

- Google Cartographer SLAM
- Real-time 2D map generation
- RViz visualization
- Map saving
- Saved map support for navigation

### 🎮 Robot Teleoperation

Keyboard-based manual robot control using:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/bumpy7/cmd_vel
```

### 🖥️ Development & Visualization

The repository contains a separate `Bumpy Rviz` directory containing:

- RViz configurations for multiple robots
- Mapping visualization
- Navigation visualization
- TF frame visualization
- Development launch files
- Camera utility

### 🤝 Multi-Robot Coordination

The workspace contains a `formation_control` package for multi-robot coordination.

Current development includes:

- 2-robot formation control
- 3-robot formation control
- Alternative 3-robot formation implementation
- Obstacle handling

---

# 🛠️ Technology Stack

| Component | Technology |
|---|---|
| Operating System | Ubuntu 22.04 |
| Middleware | ROS2 Humble |
| Programming | Python / C++ |
| SLAM | Google Cartographer |
| Navigation | ROS2 Navigation Stack |
| Visualization | RViz2 |
| Environment Perception | 2D LiDAR |
| Build System | Colcon |
| Multi-Robot Control | Python |
| Robot Communication | ROS2 Topics / TF |

---

# 📂 Project Structure

```text
bumpy_ws/
│
├── Bumpy Rviz/
│   │
│   ├── bumpy1.rviz
│   ├── bumpy2.rviz
│   ├── bumpy3.rviz
│   ├── bumpy4.rviz
│   ├── bumpy5.rviz
│   ├── bumpy6.rviz
│   ├── bumpy7.rviz
│   ├── bumpy7 (copy).rviz
│   ├── tb3.rviz
│   │
│   ├── cam.py
│   ├── robot.launch.py
│   │
│   ├── frames_2026-03-20_14.23.28.gv
│   └── frames_2026-03-20_14.23.28.pdf
│
├── src/
│   │
│   ├── bumpy_bringup/
│   │
│   ├── bumpy_sensors/
│   │
│   ├── bumpy_slam/
│   │   ├── launch/
│   │   │   ├── slam.launch.py
│   │   │   ├── cartographer.launch.py
│   │   │   ├── save_map.launch.py
│   │   │   └── sllidar_a1_launch.py
│   │   │
│   │   ├── config/
│   │   ├── maps/
│   │   ├── waypoints/
│   │   ├── bumpy_slam/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── formation_control/
│       └── formation_control/
│           ├── 2robot.py
│           ├── 3robot.py
│           ├── 3robot2.py
│           ├── obstacle.py
│           └── __init__.py
│
├── build/        # Ignored
├── install/      # Ignored
└── log/          # Ignored
```

> `__pycache__/` and generated `.pyc` files should normally be excluded from Git using `.gitignore`.

---

# ⚙️ Installation

## Prerequisites

Install the following:

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- Colcon
- RViz2
- Cartographer
- Navigation2

Install required ROS2 packages:

```bash
sudo apt update

sudo apt install \
ros-humble-cartographer \
ros-humble-cartographer-ros \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-teleop-twist-keyboard \
ros-humble-rviz2
```

---

# 📥 Clone the Repository

```bash
git clone https://github.com/rajagopal95/bumpy_ws.git
```

Move into the workspace:

```bash
cd bumpy_ws
```

---

# 🔨 Build the Workspace

Build all ROS2 packages:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

For every new terminal:

```bash
source ~/bumpy_ws/install/setup.bash
```

---

# 🤖 Robot Bringup

Start the robot bringup system:

```bash
ros2 launch bumpy_bringup bringup.launch.py
```

The bringup system is responsible for starting the required robot components such as:

- Robot drivers
- Sensors
- TF
- Robot state
- Velocity interfaces

---

# 📡 LiDAR

The robot uses a **2D LiDAR** for environment perception.

The LiDAR provides range data that is used by the SLAM system to construct a 2D representation of the environment.

The LiDAR launch file is located at:

```text
src/bumpy_slam/launch/sllidar_a1_launch.py
```

---

# 🗺️ SLAM

The project uses **Google Cartographer** for real-time SLAM.

Start SLAM using:

```bash
ros2 launch bumpy_slam slam.launch.py
```

This launches the required SLAM components and allows the robot to construct a map while moving through the environment.

---

# 🎮 Manual Teleoperation

The robot can be manually controlled using the ROS2 keyboard teleoperation package.

Run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/bumpy7/cmd_vel
```

The velocity commands are published to:

```text
/bumpy7/cmd_vel
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

Additional keys provided by `teleop_twist_keyboard` can be displayed in the terminal when the node starts.

---

# 💾 Saving a Map

After completing the SLAM process, save the generated map using:

```bash
ros2 launch bumpy_slam save_map.launch.py
```

The saved map is stored in:

```text
src/bumpy_slam/maps/
```

A typical saved map contains:

```text
maps/
├── map_name.yaml
└── map_name.pgm
```

### `.yaml`

Contains the map metadata such as:

- Map image
- Resolution
- Origin
- Occupancy thresholds

### `.pgm`

Contains the actual occupancy-grid map image.

---

# 🧭 Autonomous Navigation

After generating and saving a map, the map can be used for autonomous navigation.

The general navigation pipeline is:

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

The navigation system uses the map together with sensor data and robot pose information to plan and execute paths.

---

# 🖥️ RViz Development Tools

A separate directory named:

```text
Bumpy Rviz/
```

contains RViz configuration files used during development and testing.

## RViz Configurations

The directory currently contains configurations for multiple robots:

```text
Bumpy Rviz/
├── bumpy1.rviz
├── bumpy2.rviz
├── bumpy3.rviz
├── bumpy4.rviz
├── bumpy5.rviz
├── bumpy6.rviz
├── bumpy7.rviz
├── bumpy7 (copy).rviz
└── tb3.rviz
```

These configurations can be loaded into RViz2 to visualize different robot namespaces, sensor data, maps, navigation information, and TF frames.

---

# 👁️ Opening RViz2

Start RViz2:

```bash
rviz2
```

Then load the required `.rviz` configuration.

For example:

```text
Bumpy Rviz/bumpy7.rviz
```

RViz can be used to visualize:

- LiDAR scans
- Occupancy maps
- Robot model
- TF frames
- Robot pose
- Navigation paths
- Goals
- Costmaps
- Odometry

---

# 🌳 TF Frame Visualization

The `Bumpy Rviz` directory also contains generated TF frame information:

```text
frames_2026-03-20_14.23.28.gv
frames_2026-03-20_14.23.28.pdf
```

These files provide a graphical representation of the robot's TF tree.

A typical AMR TF structure may look like:

```text
map
 │
 ▼
odom
 │
 ▼
base_link
 │
 ├── laser
 │
 └── other sensors
```

The exact TF tree depends on the robot configuration.

---

# 📷 Camera Development Utility

The `Bumpy Rviz` directory also contains:

```text
cam.py
```

This script is used as a development utility for camera-related testing.

---

# 🚀 Development Launch File

The `Bumpy Rviz` directory contains:

```text
robot.launch.py
```

This launch file is used during development/testing of the robot visualization or simulation environment.

---

# 🤝 Multi-Robot Formation Control

The workspace contains a dedicated package:

```text
src/formation_control/
```

This package contains experimental and development code for **multi-robot coordination and formation control**.

Current implementations include:

```text
formation_control/
├── 2robot.py
├── 3robot.py
├── 3robot2.py
├── obstacle.py
└── __init__.py
```

---

# 👥 Two-Robot Formation

The file:

```text
2robot.py
```

contains the development implementation for coordinating two robots.

The objective is to allow multiple robots to maintain a defined relationship or formation while moving.

Conceptually:

```text
        Robot 1
           │
           │
           ▼
      Formation Logic
           │
           ▼
        Robot 2
```

---

# 👥👥 Three-Robot Formation

The workspace contains two implementations for three-robot coordination:

```text
3robot.py
3robot2.py
```

These files are used for experimentation with different multi-robot formation strategies.

A typical formation can be represented as:

```text
             Robot 1
                ▲
               / \
              /   \
             /     \
        Robot 2 ─── Robot 3
```

The exact formation geometry depends on the implementation.

---

# 🚧 Obstacle Handling

The file:

```text
obstacle.py
```

contains development code related to obstacle handling for multi-robot coordination.

The objective is to extend formation control so that the robot group can react to obstacles while maintaining coordinated movement.

Conceptually:

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

Future versions can combine obstacle avoidance with dynamic formation reconfiguration.

---

# 🔄 Multi-Robot System Architecture

The overall multi-robot architecture can be represented as:

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

The complete development workflow is:

```text
                         START
                           │
                           ▼
                    Robot Bringup
                           │
                           ▼
                    Sensor Startup
                           │
                           ▼
                       2D LiDAR
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
                    Localization
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

The system uses ROS2 topics and TF for communication between different components.

Important interfaces include:

### Velocity Command

```text
/bumpy7/cmd_vel
```

Used to send velocity commands to the robot.

### LiDAR

```text
/scan
```

Used for LiDAR range measurements.

Other topics and TF frames depend on the robot and sensor configuration.

To inspect available topics:

```bash
ros2 topic list
```

To inspect a topic:

```bash
ros2 topic echo /bumpy7/cmd_vel
```

To inspect the TF tree:

```bash
ros2 run tf2_tools view_frames
```

---

# 🧪 Development

During development, the `Bumpy Rviz` directory can be used for visualization and debugging.

The multi-robot development code is located in:

```text
src/formation_control/
```

This allows the core AMR stack and experimental multi-robot algorithms to remain modular.

---

# 📁 Recommended Git Ignore

Generated ROS2 build files and Python cache files should not be committed.

Recommended `.gitignore`:

```gitignore
# ROS2
build/
install/
log/

# Python
__pycache__/
*.pyc

# IDE
.vscode/
.idea/

# OS
.DS_Store
```

---

# 📸 Results

The system is designed to demonstrate:

- ✅ Real-time LiDAR mapping
- ✅ Cartographer SLAM
- ✅ Saved occupancy-grid maps
- ✅ RViz visualization
- ✅ Manual robot control
- ✅ Autonomous navigation
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
- Sensor fusion

### Perception

- LiDAR + IMU fusion
- LiDAR + Camera fusion
- AI-based object detection
- Dynamic obstacle tracking

### Multi-Robot

- Dynamic formation switching
- Leader-follower control
- Decentralized coordination
- Collision avoidance
- Multi-robot task allocation
- Swarm intelligence
- Distributed navigation

### Remote Operation

- Web-based robot monitoring
- Multi-robot dashboard
- Remote teleoperation
- Real-time status monitoring

---

# 🎯 Potential Applications

The platform can be extended for:

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

**Raja**

ROS2 | Autonomous Mobile Robots | SLAM | Navigation | Multi-Robot Systems

---

# 📜 License

This project is developed for **educational, development, and research purposes**.
