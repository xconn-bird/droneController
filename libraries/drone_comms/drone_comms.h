#pragma once
#ifndef DRONE_COMMS_H
#define DRONE_COMMS_H

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif



struct controller3 {
  int thrVal = 0;   // thrust input value
  int thrPos = 0;   // thrust input position
  int yawPos = 0;   // yaw input position
  int pitPos = 0;   // pitch input position
  int rolPos = 0;   // roll input position
  int lPush = 0;   // left joystick button position
  int rPush = 0;   // right joystick button position
  //int x_axis_3 = 0; // controller nub 3
  //int y_axis_3 = 0;   // handles camera angles
};

class drone_comms {
public:
  //RH_RF95 rf95;
  drone_comms();
  void rSetup();
  void loopComms();
  void hello();
  void receivePacket();
  void sendAckPacket();

  uint8_t data[7];
  uint8_t len;
  uint8_t key_control[2];
  uint8_t key_ack[2];
  bool ack;
  bool saidhello;
  int silence_count;
  
  controller3 Controller;

private:
  void decodeControlPacket();
  void throttleCalc();

  float scaler;
  float value;
};


#endif // !DRONE_COMMS_H
