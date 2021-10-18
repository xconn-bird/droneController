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

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
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
uint8_t radiopacket[20];
uint8_t len = 20;

int16_t packetnum = 0;  // packet counter, we increment per xmission
bool saidhello = false;

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
  delay(1);

  if (!saidhello)
    hello();
  packetnum++;
  //Serial.println(packetnum);
  if (packetnum >= 10) {
    saidhello = false;
    packetnum = 0;
  }
  
  receivePacket();
  
  /*Serial.print("Int variable values: ");
  Serial.print(Controller.thrPos);
  Serial.print(Controller.yawPos);
  Serial.print(Controller.pitPos);
  Serial.print(Controller.rolPos);
  Serial.print(Controller.lPush);
  Serial.println(Controller.rPush);*/
}

void receivePacket() {
  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(radiopacket, &len)) {
      Serial.print("Packet received: ");
      Serial.println((char*)radiopacket);
      
      decodePacket();
    }
  }
  else {
    Serial.println("NO INPUT packets from controller");
    Controller.thrPos = 0;
    Controller.yawPos = 0;
    Controller.pitPos = 0;
    Controller.rolPos = 0;
    Controller.lPush = 0;
    Controller.rPush = 0;
  }
  
}

void decodePacket() {
  Controller.thrPos = radiopacket[1]-48;
  if (radiopacket[0] == '-')
    Controller.thrPos *= -1;

  Controller.yawPos = radiopacket[3]-48;
  if (radiopacket[2] == '-')
    Controller.yawPos *= -1;

  Controller.pitPos = radiopacket[5]-48;
  if (radiopacket[4] == '-')
    Controller.pitPos *= -1;

  Controller.rolPos = radiopacket[7]-48;
  if (radiopacket[6] == '-')
    Controller.rolPos *= -1;

  Controller.lPush = radiopacket[8]-48;
  Controller.rPush = radiopacket[9]-48;
}
