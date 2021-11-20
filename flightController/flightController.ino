#include <Servo.h>
#include <mpu_sensor.h>

// function inits
void rate();
int stabilizerPID(int, int);
int ratePID(int, float);
void accumulatorPID(int, int, int);
void flightProcessor();

// Sensor Vars
float sensorYaw, sensorPitch, sensorRoll = 0;
int yawangle, pitchangle, rollangle = 0;
float yawrate, pitchrate, rollrate = 0;

// Controller Vars
int thrust = 10, cyaw = 0, cpitch = 0, croll = 0;  // thrust range = 0-100%, yaw range = (-360)-360 deg, pitch & roll range = (-45)-45 deg
int yaw, pitch, roll = 0;

// indi motor controls
int m1_pwm, m2_pwm, m3_pwm, m4_pwm = 0;

// time vars
uint32_t last_update_time = micros();
float last_update_angle_x, last_update_angle_y, last_update_angle_z = 0;

// constants
float angleConst = 1;
float rateConst = 1;

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
  croll = analogRead(A3);
  croll = map(croll, 0, 1023, 0, 180);
  cyaw = analogRead(A3);
  cyaw = map(cyaw, 0, 1023, 0, 180);
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
  accumulatorPID(cpitch, croll, cyaw);
  flightProcessor();

  Serial.print("user yaw:\t");
  Serial.println(cyaw);
  Serial.print("user pitch:\t");
  Serial.println(cpitch);
  Serial.print("user roll:\t");
  Serial.println(croll);
  Serial.println("/////////////////");

  Serial.print("final yaw:\t");
  Serial.println(yaw);
  Serial.print("final pitch:\t");
  Serial.println(pitch);
  Serial.print("final roll:\t");
  Serial.println(roll);
 
  Serial.print("Motor 1\t");
  Serial.println(m1_pwm);
  Serial.print("Motor 2\t");
  Serial.println(m2_pwm);
  Serial.print("Motor 3\t");
  Serial.println(m3_pwm);
  Serial.print("Motor 4\t");
  Serial.println(m4_pwm);
  Serial.println("/////////////////");
  
  ESC[0].write(m1_pwm);
  ESC[1].write(m2_pwm);
  ESC[2].write(m3_pwm);
  ESC[3].write(m4_pwm);

  delay(1000);
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

int altitudePID(int userthrust, float sensoralt) {
  int error = userthrust - sensoralt;
  return error;  // if error = 0, right altitude achieved, if error = -a, drone too high, if error = +a, drone too low
}



void flightProcessor() {
  thrust = map(thrust, 0, 100, 0, 200);
  yaw = map(yaw, -360, 360, 0, 60);
  pitch = map(pitch, -45, 45, 0, 60);
  roll = map(roll, -45, 45, 0, 60);
  m1_pwm = thrust + yaw + pitch + roll;
  m2_pwm = thrust - yaw + pitch - roll;
  m3_pwm = thrust - yaw - pitch + roll;
  m4_pwm = thrust + yaw - pitch - roll;
}
