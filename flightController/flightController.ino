#include <Servo.h>
#include <mpu_sensor.h>
#include <drone_comms.h>
#include <pressureSense.h>

// function inits
void rate();
int stabilizerPID(int);
void rateBalancePID(int, int &angle, int);
void accumulatorPID();
void flightProcessor();
void pitcher(int);
void yawer(int);
void roller(int);
void mixerProcessor();
int directionVector(int);
void zeroer();
void droneCalibrateHigh();
void droneCalibrateLow();

// Sensor Vars
float sensorYaw, sensorPitch, sensorRoll = 0;
int yawangle, pitchangle, rollangle = 0;
float yawrate, pitchrate, rollrate = 0;
float altitude;
float height;

// Controller Vars
int cthrust = 10, cyaw = 0, cpitch = 0, croll = 0;  // thrust range = 0-100%, yaw range = (-360)-360 deg, pitch & roll range = (-45)-45 deg
//int thrust, yaw, pitch, roll = 0;
int cthrustrate, cyawrate, cpitchrate, crollrate = 0;

// indi motor controls
int m1_pwm, m2_pwm, m3_pwm, m4_pwm = 0;

// time vars
uint32_t last_update_time = micros();
float last_update_angle_x, last_update_angle_y, last_update_angle_z = 0;
int last_cupdate_angle_w, last_cupdate_angle_x, last_cupdate_angle_y, last_cupdate_angle_z = 0;

// constants
float angleConst = 0;
float rateConst = 20;  // 20%
float maxRateConst = 5;  // 5 deg/sec

bool firsttime = true;

Servo ESC[4];
drone_comms dc;
mpu_sensor mpuSensor;
pressureSense ps;
int potVal;

void setup() {
  Serial.begin(115200);
  
  ESC[0].attach(12, 1000, 2000);
  ESC[1].attach(11, 1000, 2000);
  ESC[2].attach(10, 1000, 2000);
  ESC[3].attach(9, 1000, 2000);

  mpuSensor.mSetup();
  dc.rSetup();
  ps.mBegin();
  delay(2000);
}

void loop() {
  dc.loopComms();
  //ps.updatevalues();
  cthrust = dc.Controller.thrVal;
  cpitch = dc.Controller.pitPos;
  croll = dc.Controller.rolPos;
  cyaw = dc.Controller.yawPos;

  //croll = map(croll, 0, 1023, -45, 45);
  //cyaw = map(cyaw, 0, 1023, -360, 360);
  //cpitch = map(cpitch, 0, 1023, -45, 45);
  //cthrust = map(cthrust, 0, 1023, 0, 100);

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
    last_cupdate_angle_w = cthrust;
    last_cupdate_angle_x = cpitch;
    last_cupdate_angle_y = croll;
    last_cupdate_angle_z = cyaw;
    firsttime = false;
  } else 
    rates();
  accumulatorPID();
  
  croll = map(croll, -45, 45, 0, 80);
  cyaw = map(cyaw, -360, 360, 0, 80);
  cpitch = map(cpitch, -45, 45, 0, 80);
  cthrust = map(cthrust, 0, 100, 0, 560);
  
  flightProcessor();
  //mixerProcessor();
  
  if (dc.Controller.lPush == 1) {
    droneCalibrateHigh();
  } else if(dc.Controller.rPush == 1) {
    droneCalibrateLow();
  }
  
  Serial.print("user thrust:\t");
  Serial.println(cthrust);
  Serial.print("user yaw:\t");
  Serial.println(cyaw);
  Serial.print("user pitch:\t");
  Serial.println(cpitch);
  Serial.print("user roll:\t");
  Serial.println(croll);
  Serial.println("/////////////////");
 
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
  ESC[1].write(m2_pwm);  // + .1 * m2_pwm);
  ESC[2].write(m3_pwm);
  ESC[3].write(m4_pwm);

  //zeroer();

  //delay(1000);
}

void rates() {
  uint32_t elapsed_time = (micros() - last_update_time) / 1000000;
  yawrate = (mpuSensor.solvedYPR[0] - last_update_angle_z) / elapsed_time;
  pitchrate = (mpuSensor.solvedYPR[1] - last_update_angle_x) / elapsed_time;
  rollrate = (mpuSensor.solvedYPR[2] - last_update_angle_y) / elapsed_time;
  cthrustrate = (cthrust - last_cupdate_angle_w) / elapsed_time;
  cyawrate = (cyaw - last_cupdate_angle_z) / elapsed_time;
  cpitchrate = (cpitch - last_cupdate_angle_x) / elapsed_time;
  crollrate = (croll - last_cupdate_angle_y) / elapsed_time;

  if (yawrate < 0)
    yawrate *= -1;
  if (pitchrate < 0)
    pitchrate *= -1;
  if (rollrate < 0)
    rollrate *= -1;
  if (cthrustrate < 0)
    cthrustrate *= -1;
  if (cyawrate < 0)
    cyawrate *= -1;
  if (cpitchrate < 0)
    cpitchrate *= -1;
  if (crollrate < 0)
    crollrate *= -1;

  last_update_angle_x = mpuSensor.solvedYPR[1];
  last_update_angle_y = mpuSensor.solvedYPR[2];
  last_update_angle_z = mpuSensor.solvedYPR[0];
  last_cupdate_angle_w = cthrust;
  last_cupdate_angle_x = cpitch;
  last_cupdate_angle_y = croll;
  last_cupdate_angle_z = cyaw;
}

int stabilizerPID(int gyroAngle) {
  int error = angleConst - gyroAngle;
  return error;  // if -ve, demonstrate +ve motion on axis and vice 
}

void rateBalancePID(int inputRate, int &angle, int gyroAngle) {
  if (inputRate == 0) {
    if (stabilizerPID(gyroAngle) < 0) {
      angle = angle + 1;  // increase angle by 1 deg
    } else {
      angle = angle - 1;  // decrease angle by 1 deg
    }
  }
}

void forcedRatePID(int &angle, float gyroRate, int d) {
  if (d == 0) {
    angle = angle + (gyroRate * rateConst / 100);
  } else {
    angle = angle - (gyroRate * rateConst / 100);
  }
}

void XYbreaker(int gyroAngle, int &angle, float gyroRate) {
  if (gyroAngle > 45 || gyroAngle < -45) {
    if (gyroAngle < 0)
      forcedRatePID(angle, gyroRate, 0);
    else
      forcedRatePID(angle, gyroRate, 1);
  }
  if (gyroAngle > 60 || gyroAngle < -60) {
    while (1) delay(10);
  }
}

void Zbreaker(int gyroRate) {
  if (gyroRate >= maxRateConst) {
    while (1) delay(10);
  }
}

void accumulatorPID() {

  Zbreaker(yawrate);
  XYbreaker(pitchangle, cpitch, pitchrate);
  XYbreaker(rollangle, croll, rollrate);
  rateBalancePID(cpitchrate, cpitch, pitchangle);
  rateBalancePID(crollrate, croll, rollangle);
  rateBalancePID(cyawrate, cyaw, yawangle);
  /*
  pitch = ratePID(stabilizerPID(userpitch, pitchangle), pitchrate);
  roll = ratePID(stabilizerPID(userroll, rollangle), rollrate);
  yaw = ratePID(stabilizerPID(useryaw, yawangle), yawrate);
  */
}

int altitudePID(int userthrust, float sensoralt) {
  int error = userthrust - sensoralt;
  return error;  // if error = 0, right altitude achieved, if error = -a, drone too high, if error = +a, drone too low
}

int* worldPositionPID(int specX, int specY, float worldX, float worldY) {
  int error[2];
  error[0] = specX - worldX;
  error[1] = specY - worldY; 
  return error;
}

void flightProcessor() {
  //thrust = map(thrust, 0, 100, 0, 200);
  //yaw = map(yaw, -360, 360, 0, 60);
  //pitch = map(pitch, -45, 45, 0, 60);
  //roll = map(roll, -45, 45, 0, 60);

  // offset to offcenter controller values
  cyaw += -40;
  cpitch += -40;
  croll += -40;
  
  m1_pwm = cthrust - cyaw - cpitch + croll;
  m2_pwm = cthrust + cyaw - cpitch - croll;
  m3_pwm = cthrust - cyaw + cpitch + croll;
  m4_pwm = cthrust + cyaw + cpitch - croll;
  

  m1_pwm = map(m1_pwm, 0, 800, 0, 180);
  m2_pwm = map(m2_pwm, 0, 800, 0, 180);
  m3_pwm = map(m3_pwm, 0, 800, 0, 180);
  m4_pwm = map(m4_pwm, 0, 800, 0, 180);
  
  if (m1_pwm < 0) 
    m1_pwm = 0;
  if (m2_pwm < 0)
    m2_pwm = 0;
  if (m3_pwm < 0)
    m3_pwm = 0;
  if (m4_pwm < 0)
    m4_pwm = 0;
  if (m1_pwm > 180) 
    m1_pwm = 180;
  if (m2_pwm > 180)
    m2_pwm = 180;
  if (m3_pwm > 180)
    m3_pwm = 180;
  if (m4_pwm > 180)
    m4_pwm = 180; 
}
/*
void pitcher(int valuepitch) {
  if (valuepitch == 0) {
    m1_pwm -= pitch;
    m2_pwm -= pitch;
    m3_pwm += pitch;
    m4_pwm += pitch;
  } else {
    m1_pwm += pitch;
    m2_pwm += pitch;
    m3_pwm -= pitch;
    m4_pwm -= pitch;
  }
}

void yawer(int valueyaw) {
  if (valueyaw == 0) {
    m1_pwm -= cyaw;
    m2_pwm += cyaw;
    m3_pwm += cyaw;
    m4_pwm -= cyaw;
  } else {
    m1_pwm += cyaw;
    m2_pwm -= cyaw;
    m3_pwm -= cyaw;
    m4_pwm += cyaw;
  }
}

void roller(int valueroll) {
  if (valueroll == 0) {
    m1_pwm -= croll;
    m2_pwm += croll;
    m3_pwm -= croll;
    m4_pwm += croll;
  } else {
    m1_pwm += croll;
    m2_pwm -= croll;
    m3_pwm += croll;
    m4_pwm -= croll;
  }
}

void mixerProcessor() {
  m1_pwm, m2_pwm, m3_pwm, m4_pwm = cthrust;
  pitcher(directionVector(cpitch));
  yawer(directionVector(cyaw));
  roller(directionVector(croll));

  m1_pwm = map(m1_pwm, 0, 800, 0, 180);
  m2_pwm = map(m2_pwm, 0, 800, 0, 180);
  m3_pwm = map(m3_pwm, 0, 800, 0, 180);
  m4_pwm = map(m4_pwm, 0, 800, 0, 180);
  
  if (m1_pwm < 0) 
    m1_pwm = 0;
  if (m2_pwm < 0)
    m2_pwm = 0;
  if (m3_pwm < 0)
    m3_pwm = 0;
  if (m4_pwm < 0)
    m4_pwm = 0;
  if (m1_pwm > 180) 
    m1_pwm = 180;
  if (m2_pwm > 180)
    m2_pwm = 180;
  if (m3_pwm > 180)
    m3_pwm = 180;
  if (m4_pwm > 180)
    m4_pwm = 180; 
}
*/
void zeroer() {
  m1_pwm = 0;
  m2_pwm = 0;
  m3_pwm = 0;
  m4_pwm = 0;
}

void droneCalibrateHigh() {
  m1_pwm = 180;
  m2_pwm = 180;
  m3_pwm = 180;
  m4_pwm = 180;
}

void droneCalibrateLow() {
  m1_pwm = 0;
  m2_pwm = 0;
  m3_pwm = 0;
  m4_pwm = 0;
}

int directionVector(int value) {
  if (value < 0) {
    value = value*-1;
    return 0;
  } else {
    return 1;
  }
}
/*
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return ;
}
*/
