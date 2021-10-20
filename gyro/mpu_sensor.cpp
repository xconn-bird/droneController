#include "mpu_sensor.h"
#include <Adafruit_MPU6050.h>

mpu_sensor::mpu_sensor() {
  this->isactive = false;
  this->acceleration.x = 0;
  this->acceleration.y = 0;
  this->acceleration.z = 0;
  this->gyroscope.x = 0;
  this->gyroscope.y = 0;
  this->gyroscope.z = 0;
  this->temperature = 0;
}

bool mpu_sensor::m_begin() {
  isactive = mpu.begin();
  return isactive;
}

void mpu_sensor::setRanges(mpu6050_accel_range_t accel, mpu6050_gyro_range_t gyro, mpu6050_bandwidth_t freq) {
  // set mpu specs
    mpu.setAccelerometerRange(accel);
    mpu.setGyroRange(gyro);
    mpu.setFilterBandwidth(freq);
}

void mpu_sensor::m_reset() {
  // begin already has a reset function in the init within it
  // so just using begin should be fine
  isactive = mpu.begin();
}

void mpu_sensor::m_update() {
  if (isactive) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    raw.getraws(mpu);

    this->acceleration.x = a.acceleration.x;
    this->acceleration.y = a.acceleration.y;
    this->acceleration.z = a.acceleration.z;
    this->gyroscope.x = g.gyro.x;
    this->gyroscope.y = g.gyro.y;
    this->gyroscope.z = g.gyro.z;
    this->temperature = temp.temperature;
    this->raw_accel.x = raw._accel.x;
    this->raw_accel.y = raw._accel.y;
    this->raw_accel.z = raw._accel.z;
    this->raw_gyro.x = raw._gyro.x;
    this->raw_gyro.y = raw._gyro.y;
    this->raw_gyro.z = raw._gyro.z;
    this->raw_temp = raw._temp;
  }
}
