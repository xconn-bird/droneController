#pragma once
#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#define INTERRUPT_PIN 2

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

struct Vector3Int {
  int x, y, z;
  Vector3Int(int x=0, int y=0, int z=0) : x(x), y(y), z(z) {}
  void setVars(int x, int y, int z) 
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
};

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

class mpu_sensor {
public:
  mpu_sensor();
  void mSetup();
  bool readyCheck();
  void setGyroOffsets(int x, int y, int z);
  void setAccelOffsets(int x, int y, int z);
  void updateMPU(char updatable=' ');
  
private:
  bool getPacket();
  void getQuaternion();
  void getEuler();
  void getypr();
  void getRawAccel();
  void getRawGyro();
  void getRealAccel();
  void getWorldAccel();
  void getTeapot();        // display quaternion values in InvenSense Teapot demo format

  MPU6050 mpu;
  // MPU control/status vars
  bool dmpReady;
  uint8_t mpuIntStatus;
  uint8_t devStatus;
  uint16_t packetSize;
  uint16_t fifoCount;
  uint8_t fifoBuffer[64];
  
public:
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  Vector3Int* gyroOffset; // [x, y, z]
  Vector3Int* accelOffset;// [x, y, z]
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float solvedEuler[3];   // [psi*180/pi, theta*180/pi, phi*180/pi]  Readable Euler angles
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  float solvedYPR[3];     // [yaw*180/pi, pitch*180/pi, roll*180/pi]  Readable Yaw Pitch Roll angles

  // packet structure for InvenSense teapot demo
  uint8_t teapotPacket[14];
  uint8_t teacup;
};

#endif // !MPU_SENSOR_H
