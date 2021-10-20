#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct vector3 {
  float x, y, z;
};

class mpu_sensor {
public:
  mpu_sensor();
  bool m_begin();
  void setRanges(mpu6050_accel_range_t accel, mpu6050_gyro_range_t gyro, mpu6050_bandwidth_t freq);
  void m_reset();
  void m_update();
private:
  Adafruit_MPU6050 mpu;
  Raw raw;
  bool isactive;
public:
  vector3 acceleration;
  vector3 gyroscope;
  float temperature;
  vector3_t raw_accel;
  vector3_t raw_gyro;
  int16_t raw_temp;
};
