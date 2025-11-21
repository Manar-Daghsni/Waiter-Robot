#include "IMUSensor.h"
#include "EncoderMotor.h"
#include "UltrasonicSensor.h"
#include <Wire.h>

// ============================
// Parameters
// ============================
#define PUBLISH_HZ 20   // 20 Hz => 50 ms
#define BAUDRATE   57600

// ============================
// Objects
// ============================
IMUSensor imuSensor;
EncoderMotor encoderMotor;
UltrasonicSensor ultrasonicSensor;

String commandBuffer = "";

// Forward declaration
void publishSensorData();
void processCommand(String cmd);

void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();

  imuSensor.begin();
  encoderMotor.begin();
  ultrasonicSensor.begin();
}

void loop() {
  // -----------------------------
  // Handle incoming serial commands
  // -----------------------------
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      commandBuffer.trim();
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += c;
    }
  }

  // -----------------------------
  // Update sensors
  // -----------------------------
  imuSensor.update();                          // update IMU readings
  float yaw = imuSensor.getYaw();              // fused IMU yaw
  encoderMotor.update(yaw);                    // odometry update with yaw
  ultrasonicSensor.update();

  // -----------------------------
  // Publish at fixed rate
  // -----------------------------
  static unsigned long lastPublish = 0;
  unsigned long interval = 1000 / PUBLISH_HZ;

  if (millis() - lastPublish >= interval) {
    publishSensorData();
    lastPublish = millis();
  }
}

// ============================
// Command handler
// ============================
void processCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd == "f") {
    encoderMotor.moveForward();  Serial.println("CMD,Forward");
  } else if (cmd == "b") {
    encoderMotor.moveBackward(); Serial.println("CMD,Backward");
  } else if (cmd == "l") {
    encoderMotor.turnLeft();     Serial.println("CMD,Left");
  } else if (cmd == "r") {
    encoderMotor.turnRight();    Serial.println("CMD,Right");
  } else if (cmd == "s") {
    encoderMotor.stop();         Serial.println("CMD,Stopped");
  } else if (cmd == "o") {
    encoderMotor.printOdom();
  } else if (cmd == "u") {
    Serial.print("U,"); 
    ultrasonicSensor.sendData(); 
    Serial.println();
  }
}

// ============================
// Publish all sensor data
// ============================
void publishSensorData() {
  // --- IMU ---
  Serial.print("I,");
  imuSensor.sendData();
  Serial.println();

  // --- Odom ---
  Serial.print("O,");
  encoderMotor.printOdom();
  Serial.println();

  // --- Encoders ---
  Serial.print("E,");
  encoderMotor.printEncoderTicks();
  Serial.println();

  // --- Ultrasonic ---
  Serial.print("U,");
  ultrasonicSensor.sendData();
  Serial.println();
}
