#pragma once
#ifndef SETUP_H
#define SETUP_H

#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0

struct controller3 {
  int x_axis_1 = 0;   // controller nub 1 
  int y_axis_1 = 0;   // handles roll and pitch
  int x_axis_2 = 0;   // controller nub 2
  int y_axis_2 = 0;   // handles thrust and yaw
  int x_axis_3 = 0; // controller nub 3
  int y_axis_3 = 0;   // handles camera angles
};

#endif // !SETUP_H
