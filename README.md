# 🤖 Waiter Robot — Real Autonomous Delivery Robot  
> Full implementation: ROS 2 • Raspberry Pi 5 • Arduino • LIDAR • IMU • Ultrasonic Sensors • MQTT

<p align="center">
  <img src="docs/robot_photo.jpg" width="450px">
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge">
  <img src="https://img.shields.io/badge/Ubuntu-24.04-orange?style=for-the-badge">
  <img src="https://img.shields.io/badge/Raspberry%20Pi-5-red?style=for-the-badge">
  <img src="https://img.shields.io/badge/Arduino-UNO-blue?style=for-the-badge">
  <img src="https://img.shields.io/badge/LiDAR-RPLIDAR%20A1-yellow?style=for-the-badge">
  <img src="https://img.shields.io/badge/Navigation-Nav2-green?style=for-the-badge">
  <img src="https://img.shields.io/badge/Mapping-SLAM%20Toolbox-lightgrey?style=for-the-badge">
</p>

---

This repository contains the **complete implementation of a real autonomous waiter robot**, built using:

- **Raspberry Pi 5** running **ROS 2 Jazzy**
- **Arduino UNO/MEGA** for motor control and sensors  
- **LIDAR A1**, **IMU**, **Ultrasonic array**, **Encoders**  
- **Navigation (Nav2)**  
- **Mapping (SLAM Toolbox)**  
- **MQTT table-call system (ESP32 → ROS2)**  
- **Full robot URDF + real-world launch files**

---

## 📦 Repository Structure

```
waiter_robot/
│
├── ros2_ws_lidar/            → LIDAR + Navigation + SLAM Toolbox
│   └── src/
│       ├── config/
│       ├── maps/
│       ├── my_nav2/
│       ├── my_robot/
│       ├── scripts/
│       └── slam_toolbox/
│
├── ros2_ws_motor/            → Real robot motor + sensors control
│   └── src/
│       ├── config/
│       ├── odom_broadcaster/
│       ├── robot_call/
│       ├── robot_description/
│       └── robot_sensors/
│
└── robotcontrol/             → Arduino firmware
    ├── robotcontrol.ino
    ├── ImuSensor.h
    ├── UltrasonicSensor.h
    └── EncoderMotor.h
```

---

## 🧠 System Overview

The robot integrates these components:

### 🚗 Mobility  
- Dual DC motors with encoders  
- BTS7960 motor driver  
- Wheel odometry streamed to ROS2  

### 🧭 Sensors  
- LIDAR A1 → `/scan`  
- IMU → `/imu`  
- Ultrasonic sensors (4 directions)  
- Encoders → `/odom`  

### 🧩 Software Layers  
- Arduino → Motor control + raw sensors  
- Raspberry Pi → ROS2 Navigation Stack  
- SLAM Toolbox → Mapping  
- Nav2 → Autonomous navigation  
- MQTT → Table call system  

---

## 📡 Communication Architecture

```
Arduino → Serial → Raspberry Pi 5
     ↑                 ↓
Encoders        LIDAR / Ultrasonic / IMU
Motors          Nav2 Navigation Stack
```

MQTT:

```
ESP32 → MQTT → Raspberry Pi → robot_call package
```

---

## 🗺️ Mapping (SLAM Toolbox)

Create the map:

```bash
cd ros2_ws_lidar
source install/setup.bash
ros2 launch my_nav2 slam.launch.py
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f restaurant_map
```

---

## 🧭 Navigation (Nav2)

```bash
ros2 launch my_nav2 navigation.launch.py map:=maps/restaurant_map.yaml
```

---

## 🎛️ Motor + Sensors (Arduino → ROS2)

```bash
ros2 launch robot_sensors sensors.launch.py
ros2 launch odom_broadcaster odom.launch.py
```

---

## 📶 MQTT Table Call

```bash
ros2 run robot_call call_manager
```

---

## 📚 Documentation

Everything in the **docs/** folder:

- **hardware.md** → Wiring + components  
- **ros_architecture.md** → Full ROS2 data flow  
- **setup_and_launch.md** → How to run the robot  

---

## 📝 License

MIT License

---

## ✨ Author

**Daghsni Manar** — Robotics & Automation Engineer  
Real Waiter Robot Project  
