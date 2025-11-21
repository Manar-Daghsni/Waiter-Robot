#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <Arduino.h>
#include <math.h>

class EncoderMotor {
private:
static const int NUM_GROUPS = 2;
static const int MOTORS_PER_GROUP = 2;

const int RPWM_PINS[NUM_GROUPS] = {4, 9};
const int LPWM_PINS[NUM_GROUPS] = {6, 10};
const int REN_PINS[NUM_GROUPS]  = {7, 11};
const int LEN_PINS[NUM_GROUPS]  = {8, 12};
const int HALL_PINS[NUM_GROUPS * MOTORS_PER_GROUP] = {A2, A0, A3, A1};

struct MotorGroup {
int rpwmPin, lpwmPin;
int renPin, lenPin;
int hallPins[MOTORS_PER_GROUP];
volatile long pulseCount[MOTORS_PER_GROUP];
long prevPulseCount[MOTORS_PER_GROUP];
unsigned long lastPulseTime[MOTORS_PER_GROUP];
int prevSensorState[MOTORS_PER_GROUP];
int direction[MOTORS_PER_GROUP];
float revolutions[MOTORS_PER_GROUP];
float distance[MOTORS_PER_GROUP];
};

MotorGroup motorGroups[NUM_GROUPS];

// Encoder / wheel parameters
const unsigned long debounceDelay = 2;
const float wheelDiameter = 0.12f;       // 120 mm
const float wheelCircumference = 3.14159265f * wheelDiameter;
const float PULSES_PER_REV = 4.0f;
const float GEAR_RATIO = 1.0f;

// Robot geometry
const float L = 0.225f; // left-right distance between wheels
const float H = 0.418f; // not used for diff drive anymore

const float VEL_ALPHA = 0.45f; // smoothing

unsigned long lastUpdate = 0;
float lastTheta = 0.0f;
static constexpr float CLAMP_EPS = 1e-4f;
bool robotMoving = false;

float wheelVel[4] = {0.0f,0.0f,0.0f,0.0f};

inline float clampNoiseVal(float v) {
return (fabsf(v) < CLAMP_EPS) ? 0.0f : v;
}

void clearPrevCounts() {
for (int g = 0; g < NUM_GROUPS; ++g) {
for (int m = 0; m < MOTORS_PER_GROUP; ++m) {
motorGroups[g].prevPulseCount[m] = motorGroups[g].pulseCount[m];
motorGroups[g].lastPulseTime[m] = millis();
}
}
}

inline int wheelIndex(int g, int m) const {
if (g == 0 && m == 0) return 0; // FL
if (g == 1 && m == 0) return 1; // FR
if (g == 0 && m == 1) return 2; // RL
if (g == 1 && m == 1) return 3; // RR
return 0;
}

public:
float x = 0.0f, y = 0.0f, theta = 0.0f;
float vx = 0.0f, vy = 0.0f, vtheta = 0.0f;

EncoderMotor() {
for (int g=0; g<NUM_GROUPS; ++g){
motorGroups[g].rpwmPin = RPWM_PINS[g];
motorGroups[g].lpwmPin = LPWM_PINS[g];
motorGroups[g].renPin  = REN_PINS[g];
motorGroups[g].lenPin  = LEN_PINS[g];
for(int m=0;m<MOTORS_PER_GROUP;++m){
int index = g*MOTORS_PER_GROUP + m;
motorGroups[g].hallPins[m] = HALL_PINS[index];
motorGroups[g].pulseCount[m]=0;
motorGroups[g].prevPulseCount[m]=0;
motorGroups[g].lastPulseTime[m]=0;
motorGroups[g].prevSensorState[m]=HIGH;
motorGroups[g].revolutions[m]=0.0f;
motorGroups[g].distance[m]=0.0f;
motorGroups[g].direction[m]=0;
}
}
lastUpdate=millis();
lastTheta=theta;
}

void begin(){
for(int g=0;g<NUM_GROUPS;++g){
pinMode(motorGroups[g].rpwmPin,OUTPUT);
pinMode(motorGroups[g].lpwmPin,OUTPUT);
pinMode(motorGroups[g].renPin,OUTPUT);
pinMode(motorGroups[g].lenPin,OUTPUT);
analogWrite(motorGroups[g].rpwmPin,0);
analogWrite(motorGroups[g].lpwmPin,0);
digitalWrite(motorGroups[g].renPin,LOW);
digitalWrite(motorGroups[g].lenPin,LOW);
for(int m=0;m<MOTORS_PER_GROUP;++m){
pinMode(motorGroups[g].hallPins[m],INPUT_PULLUP);
motorGroups[g].prevSensorState[m]=digitalRead(motorGroups[g].hallPins[m]);
}
}
clearPrevCounts();
stop();
}

void moveForward()  { setMotors(150,0,1); robotMoving=true; }
void moveBackward() { setMotors(0,150,-1); robotMoving=true; }
void turnLeft()  { setMotorGroup(0,150,0,-1); setMotorGroup(1,0,150,1);  robotMoving=true; }
void turnRight() { setMotorGroup(0,0,150,1);  setMotorGroup(1,150,0,-1); robotMoving=true; }

void stop(){
setMotors(0,0,0); robotMoving=false;
clearPrevCounts(); lastUpdate=millis(); lastTheta=theta;
vx=vy=vtheta=0.0f;
for(int i=0;i<4;++i) wheelVel[i]=0.0f;
}

void setMotors(int rpwm,int lpwm,int dir){
for(int g=0;g<NUM_GROUPS;++g) setMotorGroup(g,rpwm,lpwm,dir);
}

void setMotorGroup(int g,int rpwm,int lpwm,int dir){
analogWrite(motorGroups[g].rpwmPin,rpwm);
analogWrite(motorGroups[g].lpwmPin,lpwm);
digitalWrite(motorGroups[g].renPin,HIGH);
digitalWrite(motorGroups[g].lenPin,HIGH);
for(int m=0;m<MOTORS_PER_GROUP;++m) motorGroups[g].direction[m]=dir;
}

void setPose(float x0,float y0,float theta0){
x=x0;y=y0;theta=theta0;
lastTheta=theta0; lastUpdate=millis(); vx=vy=vtheta=0.0f;
clearPrevCounts(); robotMoving=false; for(int i=0;i<4;++i) wheelVel[i]=0.0f;
}

void update(float imuYaw){
unsigned long now=millis();
float dt=(now-lastUpdate>0)?(float)(now-lastUpdate)/1000.0f:0.001f;


// read hall sensors
for(int g=0;g<NUM_GROUPS;++g){
  for(int m=0;m<MOTORS_PER_GROUP;++m){
    int pin=motorGroups[g].hallPins[m];
    int state=digitalRead(pin);
    if(state!=motorGroups[g].prevSensorState[m]){
      unsigned long nowEdge=millis();
      if(nowEdge-motorGroups[g].lastPulseTime[m]>=debounceDelay){
        int dir=motorGroups[g].direction[m];
        motorGroups[g].pulseCount[m]+=dir;
        float revs=(float)motorGroups[g].pulseCount[m]/(PULSES_PER_REV*GEAR_RATIO);
        motorGroups[g].revolutions[m]=revs;
        motorGroups[g].distance[m]=revs*wheelCircumference;
        motorGroups[g].lastPulseTime[m]=nowEdge;
      }
    }
    motorGroups[g].prevSensorState[m]=state;
  }
}

// per-wheel delta
long deltas[4]={0,0,0,0};
for(int g=0;g<NUM_GROUPS;++g){
  for(int m=0;m<MOTORS_PER_GROUP;++m){
    int idx=wheelIndex(g,m);
    long curr=motorGroups[g].pulseCount[m];
    long prev=motorGroups[g].prevPulseCount[m];
    deltas[idx]=curr-prev;
    motorGroups[g].prevPulseCount[m]=curr;
  }
}

// wheel velocities
for(int i=0;i<4;++i){
  float revs=(float)deltas[i]/(PULSES_PER_REV*GEAR_RATIO);
  float dist=revs*wheelCircumference;
  float inst_v=dist/dt;
  wheelVel[i]=VEL_ALPHA*inst_v+(1.0f-VEL_ALPHA)*wheelVel[i];
}

if(!robotMoving){ vx=vy=vtheta=0.0f; lastUpdate=now; lastTheta=theta; return; }

// === Differential drive kinematics ===
float vL = (wheelVel[0] + wheelVel[2]) * 0.5f; // left side average (FL + RL)
float vR = (wheelVel[1] + wheelVel[3]) * 0.5f; // right side average (FR + RR)

float Vx_r = (vL + vR) * 0.5f;     // forward velocity
float Vy_r = 0.0f;                 // no strafing for diff drive
float omega_r = (vR - vL) / L;     // angular velocity

float d_center = Vx_r * dt;
float d_theta  = omega_r * dt;

float encoder_theta = theta + d_theta;
const float ENCODER_WEIGHT=0.7f;
theta=ENCODER_WEIGHT*encoder_theta+(1.0f-ENCODER_WEIGHT)*imuYaw;

x += d_center * cosf(theta + 0.5f * d_theta);
y += d_center * sinf(theta + 0.5f * d_theta);

vx=clampNoiseVal(Vx_r);
vy=0.0f; // 🚨 force to zero
vtheta=clampNoiseVal((theta-lastTheta)/dt);

lastUpdate=now;
lastTheta=theta;

}

void printOdom(){
Serial.print(x,3); Serial.print(",");
Serial.print(y,3); Serial.print(",");
Serial.print(theta,3); Serial.print(",");
Serial.print(vx,3); Serial.print(",");
Serial.print(vy,3); Serial.print(",");
Serial.print(vtheta,3);
}

void printEncoderTicks(){
long fl=motorGroups[0].pulseCount[0];
long fr=motorGroups[1].pulseCount[0];
long rl=motorGroups[0].pulseCount[1];
long rr=motorGroups[1].pulseCount[1];
Serial.print(fl); Serial.print(",");
Serial.print(fr); Serial.print(",");
Serial.print(rl); Serial.print(",");
Serial.print(rr);
}

void printWheelVels(){
Serial.print("W,");
for(int i=0;i<4;++i){ Serial.print(wheelVel[i],4); if(i<3)Serial.print(","); }
Serial.println();
}
};

#endif // ENCODER_MOTOR_H
