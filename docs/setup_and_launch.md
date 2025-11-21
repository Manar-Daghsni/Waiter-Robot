# 🚀 Setup & Launch — Real Waiter Robot



---

## 🔧 1. Build all workspaces

### LIDAR + Navigation
```bash
cd ros2_ws_lidar
colcon build
source install/setup.bash
```

### Motors + Sensors
```bash
cd ros2_ws_motor
colcon build
source install/setup.bash
```

---

## 🧭 2. Run Sensors + Odometry

```bash
ros2 launch robot_sensors sensors.launch.py
ros2 launch odom_broadcaster odom.launch.py
```

Check topics:

```bash
ros2 topic list
```

---

## 🗺️ 3. Start Navigation

```bash
cd ros2_ws_lidar
ros2 launch my_nav2 navigation.launch.py map:=maps/restaurant.yaml
```

---

## 📡 4. Start MQTT Table Call System

```bash
cd ros2_ws_motor
ros2 run robot_call call_manager
```

---

## 🎮 5. Teleop (for testing)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🧪 6. Debug Tools

### See LIDAR
```bash
ros2 topic echo /scan
```

### Visualize full robot
```bash
rviz2
```

---

