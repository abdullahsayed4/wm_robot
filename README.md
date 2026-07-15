# WM Robot — Autonomous Mobile Robot for Warehouse Operations

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04-orange)](https://ubuntu.com/)

> **Graduation Project** — Faculty of Engineering at Shoubra, Benha University  
> Department of Mechanical & Mechatronics Engineering · 2024–2025  
> **Supervisor:** Dr. Mahmoud Mohamed El-Samanti

---

## Overview

WM Robot is a fully autonomous differential-drive mobile robot designed for indoor warehouse environments. It performs pickup-and-dropoff tasks without human intervention by combining embedded motor control, sensor fusion, autonomous navigation, and a web-based mission interface.

![WM Robot](docs/robot.png)

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   ROS 2 (Jazzy)                     │
│                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌───────┐  │
│  │ wm_robot_    │    │ wm_robot_    │    │ Nav2  │  │
│  │ hardware     │    │ bringup      │    │ Stack │  │
│  │ (C++)        │    │ (Launch/Cfg) │    │       │  │
│  └──────┬───────┘    └──────────────┘    └───────┘  │
│         │                                           │
│  ┌──────▼───────┐    ┌──────────────┐              │
│  │  TeensyDriver│    │  IMU Node    │              │
│  │  (Serial)    │    │  (Python)    │              │
│  └──────┬───────┘    └──────┬───────┘              │
└─────────│──────────────────│─────────────────────┘
          │                  │
    ┌─────▼──────┐    ┌──────▼───────┐    ┌──────────┐
    │ Teensy 4.1 │    │ Arduino Uno  │    │ RPLIDAR  │
    │ (PID + Enc)│    │ (BNO055 IMU  │    │    A1    │
    └────────────┘    │ + Actuator)  │    └──────────┘
                      └──────────────┘
```

---

## Hardware

| Component | Details |
|-----------|---------|
| **Chassis** | Differential-drive, custom-built |
| **Microcontroller** | Teensy 4.1 (motor PID + encoders) |
| **IMU Controller** | Arduino Uno (BNO055 IMU + linear actuator) |
| **LiDAR** | RPLIDAR A1 |
| **IMU** | BNO055 (9-DOF) |
| **Motors** | DC motors with encoders |
| **Motor Driver** | Cytron |
| **Lift Mechanism** | Scissor lift + linear actuator + limit switches |
| **Joystick** | PlayStation 4 (Bluetooth) |
| **Compute** | Raspberry Pi 4 (ROS 2 host) |

---

## Software Stack

| Layer | Technology |
|-------|-----------|
| **OS** | Ubuntu 24.04 |
| **Middleware** | ROS 2 Jazzy |
| **SLAM** | SLAM Toolbox |
| **Navigation** | Nav2 (AMCL + DWB) |
| **State Estimation** | robot_localization (EKF) |
| **Motor Control** | Custom C++ hardware interface (ros2_control) |
| **IMU Driver** | Python serial node |
| **Mission UI** | React (web-based) |

---

## Repository Structure

```
wm_robot/
├── src/
│   ├── wm_robot_project/        # URDF, meshes, Gazebo worlds, RViz
│   │   ├── urdf/                # Xacro robot description
│   │   ├── meshes/              # STL files (base, wheels, LiDAR, casters)
│   │   ├── launch/              # Display & Gazebo simulation launch
│   │   └── worlds/              # Gazebo SDF world
│   │
│   ├── wm_robot_hardware/       # C++ hardware interface & drivers
│   │   ├── src/
│   │   │   ├── robot_node.cpp               # Main ROS 2 node (50Hz control loop)
│   │   │   ├── cytron_driver.cpp            # Teensy serial driver
│   │   │   └── wm_robot_base_hardware_interface.cpp  # ros2_control plugin
│   │   ├── include/
│   │   │   ├── cytron_driver.hpp
│   │   │   └── wm_robot_base_hardware_interface.hpp
│   │   └── scripts/
│   │       └── imu_node.py      # IMU + actuator node (Arduino Uno)
│   │
│   ├── wm_robot_firmware/       # Arduino/Teensy firmware
│   │   └── firmware/
│   │       └── serial_transmitter/  # Teensy serial protocol
│   │
│   ├── wm_robot_bringup/        # Launch files & configuration
│   │   ├── launch/
│   │   │   ├── robot_launch.py       # Bring up hardware (robot + LiDAR + IMU)
│   │   │   └── navigation_launch.py  # Nav2 navigation stack
│   │   └── config/
│   │       ├── nav2_params.yaml           # Nav2 tuning
│   │       ├── ekf_config.yaml            # EKF sensor fusion
│   │       ├── wm_robot_controllers.yaml  # ros2_control config
│   │       └── Joy_stick.yaml             # PS4 joystick mapping
│   │
│   └── rplidar_ros/             # RPLIDAR A1 ROS 2 driver
│
├── maps/                        # Pre-built occupancy grid maps
│   ├── my_map1.yaml / .pgm
│   ├── my_map2.yaml / .pgm
│   └── my_map3.yaml / .pgm
│
├── start_robot.sh               # Quick startup script
└── README.md
```

---

## Getting Started

### Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop

# Dependencies
sudo apt install ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox \
                 ros-jazzy-robot-localization \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-joy \
                 ros-jazzy-teleop-twist-joy
```

### Build

```bash
mkdir -p ~/wm_robot_ws/src
cd ~/wm_robot_ws/src
git clone https://github.com/YOUR_USERNAME/wm_robot.git .
cd ~/wm_robot_ws
colcon build --symlink-install
source install/setup.bash
```

### Run

**Step 1 — Bring up the robot hardware:**
```bash
ros2 launch wm_robot_bringup robot_launch.py
```

**Step 2 — Start IMU + actuator node:**
```bash
ros2 run wm_robot_hardware imu_node
```

**Step 3 — Run SLAM (mapping mode):**
```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Step 4 — Start navigation with existing map:**
```bash
ros2 launch wm_robot_bringup navigation_launch.py \
  map:=/path/to/maps/my_map1.yaml
```

**Step 5 — Manual control (PS4 Joystick):**
```bash
ros2 launch teleop_twist_joy teleop-launch.py
```

---

## Key Features

- **Autonomous Navigation** — SLAM Toolbox for mapping, Nav2 for localization and path planning
- **Dual PID Control** — Independent PID loops per wheel (MATLAB-tuned + Arduino PID_v1)
- **Sensor Fusion** — LiDAR + IMU + wheel odometry fused via EKF
- **Scissor Lift** — Linear actuator with limit switches for precise payload handling
- **Mock Hardware Mode** — Test navigation without physical robot (`use_mock_hardware:=true`)
- **Web Mission Interface** — React-based UI for mission assignment and monitoring
- **PS4 Teleoperation** — Bluetooth joystick for manual driving during mapping

---

## Serial Protocol

### Teensy 4.1 (ttyACM0) — Motor Controller
- Baud: `115200`
- Format: `A{posL}B{posR}C{velL}D{velR}` (position in rad, velocity in rad/s)

### Arduino Uno (ttyACM1) — IMU + Actuator
- Baud: `115200`
- IMU output: `IMU,{seq},{qw},{qx},{qy},{qz},{gx},{gy},{gz},{ax},{ay},{az},{cal_sys},{cal_gyro},{cal_acc},{cal_mag}`
- Actuator commands: `1\n` (lift up) / `0\n` (lower down)
- Actuator feedback: `LIFT_DONE` / `LOWER_DONE`

---

## Team

| Name |
|------|
| Abdullah Sayed |
| Abanoub Osama |
| Elsayed Azab |
| Amir Magdy |
| Youssef Omar |
| Nourhan Nagy |

**Supervisor:** Dr. Mahmoud Mohamed El-Samanti  
**Institution:** Faculty of Engineering at Shoubra, Benha University

---

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
