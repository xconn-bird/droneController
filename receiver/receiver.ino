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
controller3 Controller;

void loop() {
  delay(1000);
  if (!saidhello)
    hello();
  receive();
  packetnum++;
  if (packetnum >= 10)
    saidhello = false;
}

void hello() {  // comm initiator block
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  while (!saidhello) {
    if (rf95.recv(buf, &len)) {
      if ((char*)buf == "thello") {
        char helloreply[8] = "rhello\n";
        rf95.send((uint8_t*)helloreply, 8);
        rf95.waitPacketSent();

        Serial.println("Hello successful. Device Transmitter and Receiver are connected!");
        saidhello = true;
      } else {
        Serial.println("Hello failed. Retrying hello...")
      }
    }
  }
}

void receive() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.recv(buf, &len)) {
    Serial.println((char*)buf);
    Controller.x_axis_1 = buf[0];
    Controller.y_axis_1 = buf[1];
    Controller.x_axis_2 = buf[2];
    Controller.y_axis_2 = buf[3];
    Controller.x_axis_3 = buf[4];
    Controller.y_axis_3 = buf[5];
  } else {
    Serial.println("Listening to for Comm");
  }
}
