#include "drone_comms.h"

//RH_RF95 rf95(RFM95_CS, RFM95_INT);
drone_comms droneComms;
// Singleton instance of the radio driver




void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  droneComms.rSetup();
}


void loop() {
  droneComms.loopComms();
}
