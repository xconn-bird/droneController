#include <SPI.h>
#include "setup.h"

void receive();

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  /*Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }*/
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

//------------------------------------------------------------------------------

controller3 Controller;
uint8_t data[3];
uint8_t len = 3;

uint8_t key_control[] = "C";
uint8_t key_ack[] = "A";

bool ack = false;
bool saidhello = false;
int silence_count = 0;

void hello() {  // comm initiator block
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

void loop() {
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
  Serial.print(Controller.thrPos);
  Serial.print(Controller.yawPos);
  Serial.print(Controller.pitPos);
  Serial.print(Controller.rolPos);
  Serial.print(Controller.lPush);
  Serial.println(Controller.rPush);
}

void receivePacket() {
  if (rf95.waitAvailableTimeout(50)) {
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

void sendAckPacket() {
  rf95.send(key_ack, 3);
  rf95.waitPacketSent();
  ///Serial.println("Sent ACKNOWLEDGMENT");
}

void decodeControlPacket() {
  Controller.lPush =   (data[1] & 0x40) >> 6;       // mask for bit 6 and shift right
  Controller.thrPos = ((data[1] & 0x38) >> 3) - 3;  // mask for bits 5-3 and shift right
  Controller.yawPos =  (data[1] & 0x07) - 3;        // mask for bits 2-0
                                                    // subtract 3 from axes to center at 0
  Controller.rPush =   (data[2] & 0x40) >> 6;       // mask for bit 6 and shift right
  Controller.pitPos = ((data[2] & 0x38) >> 3) - 3;  // mask for bits 5-3 and shift right
  Controller.rolPos =  (data[2] & 0x07) - 3;        // mask for bits 2-0
}
