#include "mpu_sensor.h"

volatile bool mpuInterrupt = false;
void dmpDataReady() {mpuInterrupt = true;}     // indicates whether MPU interrupt pin has gone high

mpu_sensor::mpu_sensor() {
  this->gyroOffset = new Vector3Int(41,8,21);
  this->accelOffset = new Vector3Int(1150,-50,1060);
  teapotPacket[0] = '$'; 
  teapotPacket[1] = 0x02;
  teapotPacket[2] = 0;
  teapotPacket[3] = 0;
  teapotPacket[4] = 0;
  teapotPacket[5] = 0;
  teapotPacket[6] = 0;
  teapotPacket[7] = 0;
  teapotPacket[8] = 0;
  teapotPacket[9] = 0;
  teapotPacket[10] = 0x00;
  teapotPacket[11] = 0x00;
  teapotPacket[12] = '\r';
  teapotPacket[13] = '\n';
  dmpReady = false;
}

void mpu_sensor::mSetup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize mpu
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // scaled for min sensitivity
  mpu.setXGyroOffset(gyroOffset->x);
  mpu.setYGyroOffset(gyroOffset->y);
  mpu.setZGyroOffset(gyroOffset->z);
  mpu.setXAccelOffset(accelOffset->x);
  mpu.setYAccelOffset(accelOffset->y);
  mpu.setZAccelOffset(accelOffset->z);

  if (devStatus == 0) {
    // calibration Time: generate offsets and calibrate mpu
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn dmp on
    mpu.setDMPEnabled(true);

    // enable interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set dmp ready flag
    dmpReady = true;

    // get dmp packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void mpu_sensor::setGyroOffsets(int x, int y, int z) {
  this->gyroOffset = new Vector3Int(x,y,z);
  mpu.setXGyroOffset(gyroOffset->x);
  mpu.setYGyroOffset(gyroOffset->y);
  mpu.setZGyroOffset(gyroOffset->z);
}

void mpu_sensor::setAccelOffsets(int x, int y, int z) {
  this->accelOffset = new Vector3Int(x,y,z);
  mpu.setXAccelOffset(accelOffset->x);
  mpu.setYAccelOffset(accelOffset->y);
  mpu.setZAccelOffset(accelOffset->z);
}

bool mpu_sensor::readyCheck() {
  return dmpReady;
}

bool mpu_sensor::getPacket() {
  return mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}

void mpu_sensor::getQuaternion() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
}

void mpu_sensor::getEuler() {
  getQuaternion();
  mpu.dmpGetEuler(euler, &q);
  this->solvedEuler[0] = euler[0]*180/M_PI;
  this->solvedEuler[1] = euler[1]*180/M_PI;
  this->solvedEuler[2] = euler[2]*180/M_PI;
}

void mpu_sensor::getypr() {
  getQuaternion();
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  this->solvedYPR[0] = ypr[0]*180/M_PI;
  this->solvedYPR[1] = ypr[1]*180/M_PI;
  this->solvedYPR[2] = ypr[2]*180/M_PI;
}

void mpu_sensor::getRawAccel() {
  getQuaternion();
  mpu.dmpGetAccel(&aa, fifoBuffer);
}

void mpu_sensor::getRawGyro() {
  getQuaternion();
  mpu.dmpGetGyro(&gy, fifoBuffer);
}

void mpu_sensor::getRealAccel() {
  getRawAccel();
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

void mpu_sensor::getWorldAccel() {
  getRealAccel();
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}

void mpu_sensor::getTeapot() {
  // display quaternion values in InvenSense Teapot demo format:
  teapotPacket[2] = fifoBuffer[0];
  teapotPacket[3] = fifoBuffer[1];
  teapotPacket[4] = fifoBuffer[4];
  teapotPacket[5] = fifoBuffer[5];
  teapotPacket[6] = fifoBuffer[8];
  teapotPacket[7] = fifoBuffer[9];
  teapotPacket[8] = fifoBuffer[12];
  teapotPacket[9] = fifoBuffer[13];
  teacup = teapotPacket[14];
  teapotPacket[11]++;  // packetCount, loops at 0xFF on purpose
}

void mpu_sensor::updateMPU(char updatable) {
  getPacket();
  switch (updatable) {
    case 'q':
      getQuaternion();
    break;
    case 'r':
      getRawAccel();
      getRawGyro();
    break;
    case 'w':
      getWorldAccel();
    break;
    default:
      getEuler();
      getypr();
      getRealAccel();
      getTeapot();
  }
}
