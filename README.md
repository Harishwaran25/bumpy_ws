# bumpy_ws
# 🤖 ROS2 Autonomous Mobile Robot (AMR) using Cartographer

## 📌 Overview

This project implements an Autonomous Mobile Robot (AMR) using ROS2 on Ubuntu 22.04.
The robot is capable of performing real-time mapping, localization, and autonomous navigation in indoor environments.

It uses Google Cartographer for SLAM and a custom navigation setup for path planning and movement.

---

## 🚀 Features

* 🗺️ Real-time SLAM using Cartographer
* 🚗 Autonomous navigation using ROS2
* 📡 LiDAR-based environment perception
* 🔁 Modular ROS2 architecture
* 🤖 Scalable for multi-robot applications

---

## 🛠️ Tech Stack

* ROS2 (Humble)
* Python & C++
* Cartographer (SLAM)
* RViz (Visualization)
* Ubuntu 22.04

---

## 📂 Project Structure

```
bumpy_ws/
 ├── src/
 │    ├── bumpy_bringup/
 │    ├── bumpy_navigation/
 │    └── (other packages)
 ├── build/        (ignored)
 ├── install/      (ignored)
 ├── log/          (ignored)
```

---

## ⚙️ Installation

### Prerequisites

* Ubuntu 22.04
* ROS2 Humble installed

### Steps

```bash
# Clone the repository
git clone https://github.com/rajagopal95

# Move into workspace
cd bumpy_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

---

## ▶️ Usage

### 1️⃣ Bringup the Robot

```bash
ros2 launch bumpy_bringup bringup.launch.py
```

### 2️⃣ Start Mapping (Cartographer)

```bash
ros2 launch bumpy_navigation cartographer.launch.py
```

### 3️⃣ Start Navigation

```bash
ros2 launch bumpy_navigation navigation.launch.py
```

---

## 🧠 System Workflow

1. Robot is initialized using the bringup package
2. Cartographer generates a real-time map using LiDAR data
3. Navigation stack uses the map to plan paths
4. Robot follows planned trajectories using velocity commands

---

## 📸 Results

* Real-time map generation in RViz
* Autonomous navigation in mapped environment

(Add screenshots here if available)

---

## 🔮 Future Improvements

* 🔗 Sensor fusion (LiDAR + Camera)
* 🧠 AI-based obstacle detection
* 🤝 Multi-robot coordination and swarm behavior
* 📍 Improved localization accuracy

---

## 👨‍💻 Author

* Raja

---

## 📜 License

This project is for educational and research purposes.
