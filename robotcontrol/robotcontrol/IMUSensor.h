// =======================
// IMUSensor.h
// =======================
#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class IMUSensor {
private:
  // --- MPU9250 ---
  static const uint8_t MPU9250_ADDR = 0x68;
  static const uint8_t WHO_AM_I     = 0x75;
  static const uint8_t PWR_MGMT_1   = 0x6B;
  static const uint8_t ACCEL_XOUT_H = 0x3B;
  static const uint8_t GYRO_XOUT_H  = 0x43;
  static const uint8_t ACCEL_CONFIG = 0x1C;
  static const uint8_t GYRO_CONFIG  = 0x1B;
  static const uint8_t SMPLRT_DIV   = 0x19;
  static const uint8_t CONFIG       = 0x1A;

  // --- AK8963 ---
  static const uint8_t AK8963_ADDR  = 0x0C;
  static const uint8_t AK8963_ST1   = 0x02;
  static const uint8_t AK8963_XOUT_L= 0x03;
  static const uint8_t AK8963_ST2   = 0x09;
  static const uint8_t AK8963_CNTL1 = 0x0A;
  static const uint8_t AK8963_ASAX  = 0x10;

  // Calibrations / filtres
  static const int   CALIBRATION_SAMPLES = 500;
  static constexpr float KALMAN_Q   = 0.1f;
  static constexpr float KALMAN_R   = 0.1f;
  static constexpr float GYRO_WEIGHT = 0.85f;
  static constexpr float MAG_WEIGHT  = 0.05f;
  static constexpr float GRAVITY_N   = 9.80665f;

  // ZUPT
  static constexpr float ZUPT_BETA    = 0.001f;
  static constexpr float ZUPT_GYR_TH  = 0.02f;
  static constexpr float ZUPT_ACC_TH  = 0.20f;

  static constexpr float MAG_DECLINATION = 0.0f;

  // État
  float gyro_bias[3]  = {0,0,0};
  float accel_bias[3] = {0,0,0};
  float accel_scale[3]= {1,1,1};
  bool  calibrated    = false;

  float lastRawGyro[3] = {0,0,0};
  float lastAccel[3]   = {0,0,0};
  float lastGyro[3]    = {0,0,0};

  float roll=0, pitch=0, yaw=0;
  unsigned long lastUpdate = 0;
  float magAdj[3] = {1,1,1};

  struct KalmanFilter { float x, P, Q, R, K; };
  KalmanFilter kalman_roll  = {0,1,KALMAN_Q,KALMAN_R,0};
  KalmanFilter kalman_pitch = {0,1,KALMAN_Q,KALMAN_R,0};
  KalmanFilter kalman_accel[3] = {
    {0,1,KALMAN_Q,KALMAN_R,0},
    {0,1,KALMAN_Q,KALMAN_R,0},
    {0,1,KALMAN_Q,KALMAN_R,0}
  };
  KalmanFilter kalman_gyro[3]  = {
    {0,1,KALMAN_Q,KALMAN_R,0},
    {0,1,KALMAN_Q,KALMAN_R,0},
    {0,1,KALMAN_Q,KALMAN_R,0}
  };

  // I2C helpers
  uint8_t readByte(uint8_t addr, uint8_t reg){
    Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0;
  }
  void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* buf){
    Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(addr, count);
    for (uint8_t i=0; i<count && Wire.available(); ++i) buf[i] = Wire.read();
  }
  void writeByte(uint8_t addr, uint8_t reg, uint8_t data){
    Wire.beginTransmission(addr); Wire.write(reg); Wire.write(data); Wire.endTransmission();
  }

  float applyKalmanFilter(KalmanFilter* f, float z){
    f->P += f->Q;
    f->K  = f->P / (f->P + f->R);
    f->x += f->K * (z - f->x);
    f->P *= (1.0f - f->K);
    return f->x;
  }

  void initMPU(){
    writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x00); delay(100);
    writeByte(MPU9250_ADDR, SMPLRT_DIV, 0x07);
    writeByte(MPU9250_ADDR, CONFIG,     0x03);
    writeByte(MPU9250_ADDR, GYRO_CONFIG,  0x00); // ±250 dps
    writeByte(MPU9250_ADDR, ACCEL_CONFIG, 0x00); // ±2 g
    delay(10);
  }

  void initAK8963(){
    writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00); delay(10);
    writeByte(AK8963_ADDR, AK8963_CNTL1, 0x0F); delay(10);
    uint8_t asa[3] = {0,0,0};
    readBytes(AK8963_ADDR, AK8963_ASAX, 3, asa);
    magAdj[0] = ((float)asa[0]-128.0f)/256.0f + 1.0f;
    magAdj[1] = ((float)asa[1]-128.0f)/256.0f + 1.0f;
    magAdj[2] = ((float)asa[2]-128.0f)/256.0f + 1.0f;
    writeByte(AK8963_ADDR, AK8963_CNTL1, 0x16); delay(10); // 100 Hz, 16-bit
  }

  void readAccelerometer(float *accel) {
    uint8_t data[6] = {0};
    readBytes(MPU9250_ADDR, ACCEL_XOUT_H, 6, data);
    int16_t ax_raw = (int16_t)((data[0]<<8)|data[1]);
    int16_t ay_raw = (int16_t)((data[2]<<8)|data[3]);
    int16_t az_raw = (int16_t)((data[4]<<8)|data[5]);

    float ax = ax_raw * 2.0f * GRAVITY_N / 32768.0f;
    float ay = ay_raw * 2.0f * GRAVITY_N / 32768.0f;
    float az = az_raw * 2.0f * GRAVITY_N / 32768.0f;

    if (calibrated){
      ax = (ax - accel_bias[0]) * accel_scale[0];
      ay = (ay - accel_bias[1]) * accel_scale[1];
      az = (az - accel_bias[2]) * accel_scale[2];
    }

    lastAccel[0] = applyKalmanFilter(&kalman_accel[0], ax);
    lastAccel[1] = applyKalmanFilter(&kalman_accel[1], ay);
    lastAccel[2] = applyKalmanFilter(&kalman_accel[2], az);

    accel[0] = lastAccel[0];
    accel[1] = lastAccel[1];
    accel[2] = lastAccel[2];
  }

  void readGyroscope(float *gyro) {
    uint8_t data[6] = {0};
    readBytes(MPU9250_ADDR, GYRO_XOUT_H, 6, data);
    const float s = (250.0f * PI / 180.0f) / 32768.0f;

    int16_t gx_raw = (int16_t)((data[0]<<8)|data[1]);
    int16_t gy_raw = (int16_t)((data[2]<<8)|data[3]);
    int16_t gz_raw = (int16_t)((data[4]<<8)|data[5]);

    lastRawGyro[0] = gx_raw * s;
    lastRawGyro[1] = gy_raw * s;
    lastRawGyro[2] = gz_raw * s;

    lastGyro[0] = applyKalmanFilter(&kalman_gyro[0], lastRawGyro[0] - gyro_bias[0]);
    lastGyro[1] = applyKalmanFilter(&kalman_gyro[1], lastRawGyro[1] - gyro_bias[1]);
    lastGyro[2] = applyKalmanFilter(&kalman_gyro[2], lastRawGyro[2] - gyro_bias[2]);

    gyro[0] = lastGyro[0];
    gyro[1] = lastGyro[1];
    gyro[2] = lastGyro[2];
  }

  bool readMagnetometer(float *mag) {
    uint8_t st1 = readByte(AK8963_ADDR, AK8963_ST1);
    if (!(st1 & 0x01)) return false;

    uint8_t data[7] = {0};
    readBytes(AK8963_ADDR, AK8963_XOUT_L, 7, data);
    uint8_t st2 = data[6];
    if (st2 & 0x08) return false;

    int16_t mx = (int16_t)((data[1]<<8)|data[0]);
    int16_t my = (int16_t)((data[3]<<8)|data[2]);
    int16_t mz = (int16_t)((data[5]<<8)|data[4]);

    mag[0] = (float)mx * 0.15f * magAdj[0];
    mag[1] = (float)my * 0.15f * magAdj[1];
    mag[2] = (float)mz * 0.15f * magAdj[2];
    return true;
  }

  void updateOrientation(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt) {
    // Intégration gyro
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    // Accel -> roll/pitch
    float accel_roll  = atan2f(ay, az);
    float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    roll  = roll  * GYRO_WEIGHT + accel_roll  * (1.0f - GYRO_WEIGHT);
    pitch = pitch * GYRO_WEIGHT + accel_pitch * (1.0f - GYRO_WEIGHT);

    // Magneto -> yaw (faible pondération)
    float mag_data[3];
    if (readMagnetometer(mag_data)) {
      float mag_yaw = atan2f(-mag_data[1], mag_data[0]) + MAG_DECLINATION;
      if (mag_yaw >  PI) mag_yaw -= 2.0f*PI;
      if (mag_yaw < -PI) mag_yaw += 2.0f*PI;
      yaw = yaw * (1.0f - MAG_WEIGHT) + mag_yaw * MAG_WEIGHT;
    }

    // Wrap
    if (roll  >  PI) roll  -= 2.0f*PI; if (roll  < -PI) roll  += 2.0f*PI;
    if (pitch >  PI) pitch -= 2.0f*PI; if (pitch < -PI) pitch += 2.0f*PI;
    if (yaw   >  PI) yaw   -= 2.0f*PI; if (yaw   < -PI) yaw   += 2.0f*PI;
  }

public:
  void begin(){
    Wire.begin();
    initMPU();
    initAK8963();
    delay(50);
    lastUpdate = millis();
  }

  void calibrate(){
    float sumg[3] = {0,0,0};
    float suma[3] = {0,0,0};
    for (int i=0; i<CALIBRATION_SAMPLES; ++i){
      float a[3], g[3];
      readAccelerometer(a);
      readGyroscope(g);
      for (int j=0;j<3;++j){ suma[j]+=a[j]; sumg[j]+=g[j]; }
      delay(5);
    }
    for (int j=0;j<3;++j){
      accel_bias[j] = suma[j] / (float)CALIBRATION_SAMPLES;
      gyro_bias[j]  = sumg[j]  / (float)CALIBRATION_SAMPLES;
    }
    calibrated = true;
    roll = pitch = yaw = 0.0f;
  }

  void update(){
    float a[3], g[3];
    readAccelerometer(a);
    readGyroscope(g);

    unsigned long now = millis();
    float dt = (lastUpdate==0) ? 0.01f : (now - lastUpdate)/1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
    lastUpdate = now;

    // ZUPT simple
    if (isStill(a, g)){
      for (int i=0;i<3;++i){
        gyro_bias[i] = (1.0f - ZUPT_BETA) * gyro_bias[i] + ZUPT_BETA * lastRawGyro[i];
      }
    }

    updateOrientation(lastGyro[0], lastGyro[1], lastGyro[2],
                      lastAccel[0], lastAccel[1], lastAccel[2], dt);
  }

  void getIMUData(float* accel, float* gyro){
    accel[0]=lastAccel[0]; accel[1]=lastAccel[1]; accel[2]=lastAccel[2];
    gyro[0]=lastGyro[0];   gyro[1]=lastGyro[1];   gyro[2]=lastGyro[2];
  }

  void getQuaternion(double* q){
    double cy = cos((double)yaw * 0.5);
    double sy = sin((double)yaw * 0.5);
    double cp = cos((double)pitch * 0.5);
    double sp = sin((double)pitch * 0.5);
    double cr = cos((double)roll * 0.5);
    double sr = sin((double)roll * 0.5);

    q[3] = cr*cp*cy + sr*sp*sy; // w
    q[0] = sr*cp*cy - cr*sp*sy; // x
    q[1] = cr*sp*cy + sr*cp*sy; // y
    q[2] = cr*cp*sy - sr*sp*cy; // z

    double norm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (norm > 1e-12){ q[0]/=norm; q[1]/=norm; q[2]/=norm; q[3]/=norm; }
    else { q[0]=q[1]=q[2]=0; q[3]=1; }
  }

  // ✅ Added sendData from your old version
  void sendData() {
    float accel[3], gyro[3];
    double quat[4];
    getIMUData(accel, gyro);
    getQuaternion(quat);

   
    Serial.print(accel[0], 3); Serial.print(",");
    Serial.print(accel[1], 3); Serial.print(",");
    Serial.print(accel[2], 3); Serial.print(",");
    Serial.print(gyro[0], 3);  Serial.print(",");
    Serial.print(gyro[1], 3);  Serial.print(",");
    Serial.print(gyro[2], 3);  Serial.print(",");
    Serial.print(quat[0], 3);  Serial.print(",");
    Serial.print(quat[1], 3);  Serial.print(",");
    Serial.print(quat[2], 3);  Serial.print(",");
    Serial.print(quat[3], 3);
  }

  float getYaw()   const { return yaw; }
  float getRoll()  const { return roll; }
  float getPitch() const { return pitch; }

private:
  bool isStill(const float* accel, const float* gyro){
    float am = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    bool acc_ok = fabsf(am - GRAVITY_N) < ZUPT_ACC_TH;
    bool gyr_ok = (fabsf(gyro[0]) < ZUPT_GYR_TH) && (fabsf(gyro[1]) < ZUPT_GYR_TH) && (fabsf(gyro[2]) < ZUPT_GYR_TH);
    return acc_ok && gyr_ok;
  }
};

#endif // IMU_SENSOR_H
