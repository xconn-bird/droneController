#include "drone_commstest.h"

drone_comms droneComms;

void setup() {
  /*Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }*/
  
  droneComms.rSetup();
}


void loop() {
  droneComms.loopComms();
}
