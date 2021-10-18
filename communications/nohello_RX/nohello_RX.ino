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

int16_t packetnum = 0;  // packet counter, we increment per xmission
bool saidhello = false;
int packetnum_d = 0;

controller3 Controller;

uint8_t radiopacket[20];
uint8_t len = 20;

void loop() {
  delay(1);
  receivePacket();
  
  Serial.print("Int variable values: ");
  Serial.print(Controller.thrPos);
  Serial.print(Controller.yawPos);
  Serial.print(Controller.pitPos);
  Serial.print(Controller.rolPos);
  Serial.print(Controller.lPush);
  Serial.println(Controller.rPush);
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
