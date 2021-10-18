#pragma once
#ifndef SETUP_H
#define SETUP_H

#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0

struct controller3 {
  int thrPos = 0;   // thrust input position
  int yawPos = 0;   // yaw input position
  int pitPos = 0;   // pitch input position
  int rolPos = 0;   // roll input position
  int lPush = 0;   // left joystick button position
  int rPush = 0;   // right joystick button position
  //int x_axis_3 = 0; // controller nub 3
  //int y_axis_3 = 0;   // handles camera angles
};

#endif // !SETUP_H
