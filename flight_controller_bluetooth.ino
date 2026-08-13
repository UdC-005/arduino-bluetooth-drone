#include <Wire.h>
#include <SoftwareSerial.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define ACCEL_XOUT_H 0x3B

const int MOTOR_FL = 5;
const int MOTOR_FR = 6;
const int MOTOR_RL = 9;
const int MOTOR_RR = 10;

SoftwareSerial bt(2, 3);   // RX, TX

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

float accelRoll, accelPitch;
float gyroRoll, gyroPitch;
float roll = 0, pitch = 0;

float rollKp = 2.0,  rollKi = 0.0,  rollKd = 0.5;
float pitchKp = 2.0, pitchKi = 0.0, pitchKd = 0.5;

float rollError, pitchError;
float rollIntegral = 0, pitchIntegral = 0;
float previousRollError = 0, previousPitchError = 0;

// These two are now LIVE setpoints, driven by Bluetooth instead of fixed at 0
float targetRoll = 0;
float targetPitch = 0;

int throttle = 0;
const int throttleStep = 20;
const int maxLeanAngle = 15;      // max degrees of tilt allowed from L/R
const unsigned long updateInterval = 200;
unsigned long lastUpdate = 0;

char lastCmd = 'S';

unsigned long previousTime = 0;
float dt;

void setup() {
  Serial.begin(115200);
  bt.begin(9600);

  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_RL, OUTPUT);
  pinMode(MOTOR_RR, OUTPUT);
  setMotors(0, 0, 0, 0);

  Wire.begin();
  writeMPU(MPU6050_ADDR, PWR_MGMT_1, 0x00);
  writeMPU(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
  writeMPU(MPU6050_ADDR, GYRO_CONFIG, 0x00);
  delay(100);

  previousTime = micros();
  Serial.println("Bluetooth-controlled self-leveling quadcopter started");
}

void loop() {
  unsigned long currentTime = micros();
  dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  readBluetooth();
  applyCommand();

  readMPU();
  calculateAngles(dt);

  float rollCorrection  = calculateRollPID(dt);
  float pitchCorrection = calculatePitchPID(dt);

  int motorFL = throttle + pitchCorrection + rollCorrection;
  int motorFR = throttle + pitchCorrection - rollCorrection;
  int motorRL = throttle - pitchCorrection + rollCorrection;
  int motorRR = throttle - pitchCorrection - rollCorrection;

  motorFL = constrain(motorFL, 0, 255);
  motorFR = constrain(motorFR, 0, 255);
  motorRL = constrain(motorRL, 0, 255);
  motorRR = constrain(motorRR, 0, 255);

  setMotors(motorFL, motorFR, motorRL, motorRR);

  Serial.print("Throttle: "); Serial.print(throttle);
  Serial.print("  Roll: "); Serial.print(roll);
  Serial.print("  Pitch: "); Serial.println(pitch);

  delay(2);
}

void readBluetooth() {
  if (bt.available()) {
    lastCmd = bt.read();
    Serial.println(lastCmd);
  }
}

// Bluetooth now only adjusts throttle and target lean angle instead of
// driving motors directly . The PID loop
// above still owns the motors.
void applyCommand() {
  if (millis() - lastUpdate < updateInterval) return;
  lastUpdate = millis();

  switch (lastCmd) {
    case 'F':
      throttle += throttleStep;
      if (throttle > 255) throttle = 255;
      targetRoll = 0;
      break;

    case 'B':
      throttle -= throttleStep;
      if (throttle < 0) throttle = 0;
      targetRoll = 0;
      break;

    case 'L':
      targetRoll = -maxLeanAngle;
      break;

    case 'R':
      targetRoll = maxLeanAngle;
      break;

    case 'S':
      targetRoll = 0;
      targetPitch = 0;
      break;
  }
}

void readMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void calculateAngles(float dt) {
  float ax = accX / 16384.0;
  float ay = accY / 16384.0;
  float az = accZ / 16384.0;

  accelRoll  = atan2(ay, az) * 180.0 / PI;
  accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  float gx = gyroX / 131.0;
  float gy = gyroY / 131.0;

  gyroRoll  = roll  + gx * dt;
  gyroPitch = pitch + gy * dt;

  roll  = 0.98 * gyroRoll  + 0.02 * accelRoll;
  pitch = 0.98 * gyroPitch + 0.02 * accelPitch;
}

float calculateRollPID(float dt) {
  rollError = targetRoll - roll;
  rollIntegral += rollError * dt;
  rollIntegral = constrain(rollIntegral, -50, 50);
  float derivative = (rollError - previousRollError) / dt;
  previousRollError = rollError;
  return rollKp * rollError + rollKi * rollIntegral + rollKd * derivative;
}

float calculatePitchPID(float dt) {
  pitchError = targetPitch - pitch;
  pitchIntegral += pitchError * dt;
  pitchIntegral = constrain(pitchIntegral, -50, 50);
  float derivative = (pitchError - previousPitchError) / dt;
  previousPitchError = pitchError;
  return pitchKp * pitchError + pitchKi * pitchIntegral + pitchKd * derivative;
}

void setMotors(int fl, int fr, int rl, int rr) {
  analogWrite(MOTOR_FL, fl);
  analogWrite(MOTOR_FR, fr);
  analogWrite(MOTOR_RL, rl);
  analogWrite(MOTOR_RR, rr);
}

void writeMPU(byte address, byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
