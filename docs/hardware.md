# 🔌 Hardware Documentation — Real Waiter Robot

This document describes ALL electronic components, wiring, and low-level hardware modules.

---

## 🛠️ Components Used
- Raspberry Pi 5 (ROS2 Humble)
- Arduino Uno/Mega
- LIDAR A1
- IMU (GY-9250)
- 4 × HC-SR04 Ultrasonic sensors
- BTS7960 motor driver
- 2 × DC motors with encoders
- 12V → 5V buck converter
- 2 × LiPo batteries
- ESP32 (MQTT Call System)

---

## ⚙️ Arduino Responsibilities
- Motor PWM control
- Encoder pulse counting
- IMU reading (I2C)
- Ultrasonic sensors (Trig/Echo)
- Sends data to Raspberry Pi through Serial

Arduino firmware files:
```
robotcontrol/
├── robotcontrol.ino
├── ImuSensor.h
├── UltrasonicSensor.h
└── EncoderMotor.h
```

---

## 🔗 Wiring Summary
- Motors → BTS7960 → Arduino PWM
- Encoders → Arduino digital pins
- IMU → Arduino SDA/SCL
- Ultrasonics → Arduino trig/echo pins
- Arduino → USB → Raspberry Pi
- LIDAR → USB → Raspberry Pi

---

## 🔋 Power Architecture
- Motors powered from separate 12V battery
- Electronics powered from 5V converter

