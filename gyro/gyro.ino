#include "mpu_sensor.h"

mpu_sensor mpuSensor;

uint32_t last_update_time = micros();
float last_update_angle_x = 0;
float last_update_angle_y = 0;
float last_update_angle_z = 0;

void rates(float sensorItem);

float yaw, pitch, roll = 0;

bool firsttime = true;

void setup() {
  Serial.begin(115200);
  while(!Serial)
    delay(10);

  mpuSensor.mSetup();
}

void loop() {
  if (!mpuSensor.readyCheck()) return;
  mpuSensor.updateMPU();
  Serial.print("ypr\t");
  Serial.print(mpuSensor.solvedYPR[0]);
  Serial.print("\t");
  Serial.print(mpuSensor.solvedYPR[1]);
  Serial.print("\t");
  Serial.print(mpuSensor.solvedYPR[2]);
  Serial.println();

  if (firsttime) {
    last_update_time = micros();
    last_update_angle_x = mpuSensor.solvedYPR[1];
    last_update_angle_y = mpuSensor.solvedYPR[2];
    last_update_angle_z = mpuSensor.solvedYPR[0];
    firsttime = false;
  } else 
    rates();

  Serial.print("ypr rates\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.println();

  delay(1000);
}

void rates() {
  uint32_t elapsed_time = (micros() - last_update_time) / 1000000;
  yaw = (mpuSensor.solvedYPR[0] - last_update_angle_z) / elapsed_time;
  pitch = (mpuSensor.solvedYPR[1] - last_update_angle_x) / elapsed_time;
  roll = (mpuSensor.solvedYPR[2] - last_update_angle_y) / elapsed_time;

  if (yaw < 0)
    yaw *= -1;
  if (pitch < 0)
    pitch *= -1;
  if (roll < 0)
    roll *= -1;

  last_update_angle_x = mpuSensor.solvedYPR[1];
  last_update_angle_y = mpuSensor.solvedYPR[2];
  last_update_angle_z = mpuSensor.solvedYPR[0];
}
