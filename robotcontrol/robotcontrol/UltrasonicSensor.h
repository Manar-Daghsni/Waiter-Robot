// =======================
// UltrasonicSensor.h
// =======================
#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
private:
  static const int NUM_SENSORS = 2; // front + back
  const int TRIG_PINS[NUM_SENSORS] = {28, 30}; // front, back
  const int ECHO_PINS[NUM_SENSORS] = {29, 31}; // front, back

  struct SensorData {
    int trigPin, echoPin;
    float distance_cm; // dernière mesure en cm (-1 si invalide)
  };

  SensorData sensors[NUM_SENSORS];

  // Round-robin non-bloquant
  int rr_index = 0;
  unsigned long lastMeasureMs = 0;
  static const unsigned long MEASURE_PERIOD_MS = 50; // ~20 Hz / 2 = 10 Hz chacun
  static const unsigned long PULSE_TIMEOUT_US = 12000; // 12 ms (~2 m)

public:
  UltrasonicSensor() {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      sensors[i].distance_cm = -1.0f;
    }
  }

  void begin() {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      sensors[i].trigPin = TRIG_PINS[i];
      sensors[i].echoPin = ECHO_PINS[i];
      pinMode(sensors[i].trigPin, OUTPUT);
      pinMode(sensors[i].echoPin, INPUT);
      digitalWrite(sensors[i].trigPin, LOW);
    }
  }

  // Appel court : ne mesure qu’UN capteur par appel
  void update() {
    unsigned long now = millis();
    if (now - lastMeasureMs < MEASURE_PERIOD_MS) return;
    lastMeasureMs = now;

    int i = rr_index;
    rr_index = (rr_index + 1) % NUM_SENSORS;

    // impulsion
    digitalWrite(sensors[i].trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensors[i].trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensors[i].trigPin, LOW);

    // lecture (timeout court)
    unsigned long dur = pulseIn(sensors[i].echoPin, HIGH, PULSE_TIMEOUT_US);
    if (dur == 0) {
      sensors[i].distance_cm = -1.0f; // hors portée
    } else {
      // 0.0343 cm/us ; aller-retour => /2
      sensors[i].distance_cm = (float)dur * 0.0343f * 0.5f;
    }
  }

  // imprime 2 distances en cm (ou -1.00 si invalide) séparées par virgules
  void sendData() {
    for (int i = 0; i < NUM_SENSORS; ++i) {
      Serial.print(sensors[i].distance_cm, 2);
      if (i < NUM_SENSORS - 1) Serial.print(",");
    }
  }
};

#endif // ULTRASONIC_SENSOR_H
