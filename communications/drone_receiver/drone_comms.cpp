#include "drone_comms.h"



drone_comms::drone_comms() {
  this->rf95 = new RH_RF95(RFM95_CS, RFM95_INT);
  //rf95(RFM95_CS, RFM95_INT);
  
  len = 7;
  
  key_control[0] = 0x43;
  key_ack[0] = 0x41;
  
  ack = false;
  saidhello = false;
  silence_count = 0;
  
  scaler = 0;
  value = 0;
}

void drone_comms::rSetup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual rst
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Lora init Failed");
    while (1);
  }
  Serial.println("Lora inti Successful");
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);  // transmitter power
}


void drone_comms::loopComms() {
  if (!saidhello)
    hello();

  receivePacket();
  delay(10);
  sendAckPacket();

  if (silence_count >= 5) {
    saidhello = false;
    Controller.thrPos = 0;
    Controller.yawPos = 0;
    Controller.pitPos = 0;
    Controller.rolPos = 0;
    Controller.lPush = 0;
    Controller.rPush = 0;
  }

  Serial.print("Int variable values: ");
  Serial.print(Controller.thrPos); Serial.print(", ");
  Serial.print(Controller.yawPos); Serial.print(", ");
  Serial.print(Controller.pitPos); Serial.print(", ");
  Serial.print(Controller.rolPos); Serial.print(", ");
  Serial.print(Controller.lPush); Serial.print(", ");
  Serial.print(Controller.rPush); Serial.print(", thrVal: ");
  Serial.println(Controller.thrVal);
}

void drone_comms::hello() {  // comm initiator block
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t expected[3] = "t\n";
  char reply[3] = "r\n";
  
  while (!saidhello) {
    // receive from controller
    if (rf95.waitAvailableTimeout(1000)) {
      if (rf95.recv(buf, &len)) {
        if (buf[0] == expected[0] && buf[1] == expected[1]) {
          Serial.println("received hello from controller");

          // send to controller
          rf95.send((uint8_t*)reply, 3);
          rf95.waitPacketSent();
          Serial.println("sent hello reply");
          
          Serial.println("Hello successful. Device Transmitter and Receiver are connected!");
          saidhello = true;
        }
        else {
          Serial.println("Hello failed. Retrying hello...");
        }
      }      
    }
    else {
      // NO SIGNAL
      Serial.println("NO HELLO from controller");
    }
  }
}

void drone_comms::receivePacket() {
  if (rf95.waitAvailableTimeout(100)) {  //-----------------------------------CHANGE BACK TO 50
    if (rf95.recv(data, &len)) {
      //Serial.print("Packet received: ");
      //Serial.println((char*)data);
      
      if (data[0] == key_control[0]) {
        //Serial.println("Received a control packet");
        silence_count = 0;
        decodeControlPacket();
      }

      if (data[0] == key_ack[0]) {
        //Serial.println("Received an acknowledgement packet");
        silence_count = 0;
        hello();
      }
    }
  }
  else {
    silence_count++;
    Serial.print("Radio silence from controller: ");
    Serial.println(silence_count);
  }
}

void drone_comms::sendAckPacket() {
  rf95.send(key_ack, 3);
  rf95.waitPacketSent();
  ///Serial.println("Sent ACKNOWLEDGMENT");
}

void drone_comms::decodeControlPacket() {
  Controller.lPush =   (data[1] & 0x10) >> 4;       // mask for bit 6 and shift right
  Controller.thrPos = ((data[1] & 0x0F) << 6) | ((data[2] & 0xFC) >> 2);  // mask for bits 5-3 and shift right
  Controller.yawPos =  ((data[2] & 0x03) << 8) | (data[3]);        // mask for bits 2-0
                                                    // subtract 3 from axes to center at 0
  Controller.rPush =   (data[4] & 0x10) >> 4;       // mask for bit 6 and shift right
  Controller.pitPos = ((data[4] & 0x0F) << 6) | ((data[5] & 0xFC) >> 2);  // mask for bits 5-3 and shift right
  Controller.rolPos =  ((data[5] & 0x03) << 8) | (data[6]);        // mask for bits 2-0

  Controller.thrPos = map(Controller.thrPos,0,1024,-100,101);
  Controller.yawPos = map(Controller.yawPos,0,1024,-359,360);
  Controller.pitPos = map(Controller.pitPos,0,1024,-45,46);
  Controller.rolPos = map(Controller.rolPos,0,1024,-45,46);

  throttleCalc();    
}

void drone_comms::throttleCalc() {
  scaler = 0.05 * Controller.thrPos;
  value = value + scaler;
    
  if (value < 0) {
    value = 0;
  }
  if (value > 100) {
    value = 100;
  }
  Controller.thrVal = value;
}
