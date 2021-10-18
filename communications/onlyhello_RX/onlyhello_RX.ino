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
  delay(1000);
  if (!saidhello)
    hello();
  
  packetnum++;
  Serial.println(packetnum);
  
  if (packetnum >= 10) {
    saidhello = false;
    packetnum = 0;
  }
  
  
  //receive();
  
}

/*
void receive() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.recv(buf, &len)) {
    Serial.println((char*)buf);
  }
  else {
    Serial.println("Listening to for Comm");
  }
}*/
