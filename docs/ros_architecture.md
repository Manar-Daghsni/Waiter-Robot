# 🧠 ROS 2 Architecture — Real Waiter Robot

This document explains all ROS 2 packages and data flow.

---

## 📦 Workspace 1: ros2_ws_lidar
For mapping + navigation.

### Includes:
- `my_nav2/` → Nav2 config + launch
- `my_robot/` → LIDAR driver, SLAM
- `slam_toolbox/` → Mapping tools
- `maps/` → Saved maps

---

## 📦 Workspace 2: ros2_ws_motor
For real robot motors + sensors.

### Packages:
### 🔧 robot_description
- URDF of the real robot  
- `robot.urdf.xacro`  
- TF links: base, wheels, sensors  

### ⚙️ odom_broadcaster
- Reads encoder data from Arduino  
- Publishes:
  - `/odom`
  - TF: `odom → base_link`

### 🛰️ robot_sensors
- Publishes:
  - `/imu`
  - `/ultrasonic_front`, `/ultrasonic_left`, `/ultrasonic_right`
- Launches sensor drivers

### 📡 robot_call
- MQTT table call system
- Node: `call_manager.py`

---

## 📡 Data Flow Diagram

```
Arduino → /odom → odom_broadcaster → Nav2
Arduino → /imu → robot_sensors → Nav2
Arduino → /ultrasonic/* → robot_sensors → Costmap
LIDAR → /scan → Nav2 → Path planning
MQTT → robot_call → Behavior manager
```

---

## 🗺️ Navigation
Uses:
- SLAM Toolbox for map creation
- Nav2 for autonomous navigation
- AMCL for localization
