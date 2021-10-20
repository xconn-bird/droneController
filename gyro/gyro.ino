#include "mpu_sensor.h"

mpu_sensor mpuSensor;

float roll, pitch, rollF, pitchF, roll_offset, pitch_offset = 0;
float lp_p, lp_r = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial)
    delay(10);

  if(!mpuSensor.m_begin()) {
    Serial.println("mpu failed to init!");
    while(1) {
      delay(10);
    }
  } 
  Serial.println("mpu init successful");
  mpuSensor.setRanges(MPU6050_RANGE_2_G, MPU6050_RANGE_2000_DEG, MPU6050_BAND_21_HZ);
  delay(100);
  pitch_offset = 3.03;
  roll_offset = 1.83;
}

void loop() {
  mpuSensor.m_update();
  //Serial.print("Acceleration X: ");
  //Serial.print(mpuSensor.acceleration.x);
  //Serial.print(", Y: ");
  //Serial.print(mpuSensor.acceleration.y);
  //Serial.print(", Z: ");
  //Serial.print(mpuSensor.acceleration.z);
  //Serial.println(" m/s^2");
  roll = atan(mpuSensor.acceleration.y / sqrt(pow(mpuSensor.acceleration.x, 2) + pow(mpuSensor.acceleration.z, 2))) * 180 / PI;
  pitch = atan(-1 * mpuSensor.acceleration.x / sqrt(pow(mpuSensor.acceleration.y, 2) + pow(mpuSensor.acceleration.z, 2))) * 180 / PI;
  rollF = roll + roll_offset;
  pitchF = pitch + pitch_offset;
  lp_r = 0.94 * lp_r + 0.06 * rollF;
  lp_p = 0.94 * lp_p + 0.06 * pitchF;
  //Serial.print("roll: ");
  Serial.print(lp_r);
  Serial.print("/");
  //Serial.print("pitch: ");
  Serial.println(lp_p);
/*
  Serial.print("Rotation X: ");
  Serial.print(mpuSensor.raw_gyro.x);
  Serial.print(", Y: ");
  Serial.print(mpuSensor.raw_gyro.y);
  Serial.print(", Z: ");
  Serial.print(mpuSensor.raw_gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(mpuSensor.raw_temp);
  Serial.println(" degC");
*/
  //Serial.println("");
  //delay(1000);
}
