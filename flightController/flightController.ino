#include <Servo.h>
#include <mpu_sensor.h>

// function inits
void rate();
int stabilizerPID(int, int);
int ratePID(int, float);
void accumulatorPID(int, int, int);
void flightProcessor(int, int, int);

// Sensor Vars
float sensorYaw, sensorPitch, sensorRoll = 0;
int yawangle, pitchangle, rollangle = 0;
float yawrate, pitchrate, rollrate = 0;

// Controller Vars
int thrust, cyaw, cpitch, croll = 0;

// indi motor controls
int m1_pwm, m2_pwm, m3_pwm, m4_pwm = 0;

// time vars
uint32_t last_update_time = micros();
float last_update_angle_x, last_update_angle_y, last_update_angle_z = 0;

bool firsttime = true;

Servo ESC[4];
mpu_sensor mpuSensor;
int potVal;

void setup() {
  Serial.begin(115200);
 
  ESC[0].attach(10, 1000, 2000);
  ESC[1].attach(9, 1000, 2000);
  ESC[2].attach(8, 1000, 2000);
  ESC[3].attach(7, 1000, 2000);

  mpuSensor.mSetup();
}

void loop() {
  if (mpuSensor.readyCheck())
    mpuSensor.updateMPU();
  sensorYaw = mpuSensor.solvedYPR[0];
  sensorPitch = mpuSensor.solvedYPR[1];
  sensorRoll = mpuSensor.solvedYPR[2];
  yawangle = sensorYaw;
  pitchangle = sensorPitch;
  rollangle = sensorRoll;

  if (firsttime) {
    last_update_time = micros();
    last_update_angle_x = mpuSensor.solvedYPR[1];
    last_update_angle_y = mpuSensor.solvedYPR[2];
    last_update_angle_z = mpuSensor.solvedYPR[0];
    firsttime = false;
  } else 
    rates();
  
  potVal = analogRead(A1);
  potVal = map(potVal, 0, 1023, 0, 180);
  
  ESC[0].write(m1_pwm);
  ESC[1].write(m2_pwm);
  ESC[2].write(m3_pwm);
  ESC[3].write(m4_pwm);
  
  Serial.print("Potentiometer Value af: ");
  Serial.println(potVal);
}

void rates() {
  uint32_t elapsed_time = (micros() - last_update_time) / 1000000;
  yawrate = (mpuSensor.solvedYPR[0] - last_update_angle_z) / elapsed_time;
  pitchrate = (mpuSensor.solvedYPR[1] - last_update_angle_x) / elapsed_time;
  rollrate = (mpuSensor.solvedYPR[2] - last_update_angle_y) / elapsed_time;

  if (yaw < 0)
    yawrate *= -1;
  if (pitch < 0)
    pitchrate *= -1;
  if (roll < 0)
    rollrate *= -1;

  last_update_angle_x = mpuSensor.solvedYPR[1];
  last_update_angle_y = mpuSensor.solvedYPR[2];
  last_update_angle_z = mpuSensor.solvedYPR[0];
}

int stabilizerPID(int inputAngle, int gyroAngle) {
  int error = gyroAngle - inputAngle;
  return error * angleConst;
}

int ratePID(int inputRate, float gyroRate) {
  int error = gyroRate - inputRate;
  return error * rateConst;
}

void accumulatorPID(int userpitch, int userroll, int useryaw) {
  pitch = ratePID(stabilizerPID(userpitch, pitchangle), pitchrate);
  roll = ratePID(stabilizerPID(userroll, rollangle), rollrate);
  yaw = ratePID(stabilizerPID(useryaw, yawangle), yawrate);
}

void flightProcessor(int userpitch, int userroll, int useryaw) {
  m1_pwm = thrust - pitch - roll - yaw;
  m2_pwm = thrust + pitch + roll - yaw;
  m3_pwm = thrust + pitch - roll + yaw;
  m4_pwm = thrust - pitch + roll + yaw;
}
