#include <SPI.h>
#include <RH_RF95.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

 // for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// controller input pins
#define joy1X A0 // throttle
#define joy1Y A1 // yaw
#define joy2X A2 // pitch
#define joy2Y A3 // roll
#define joy1button 10 // left button
#define joy2button 11 // right button

void setup() 
{
  pinMode(joy1button, INPUT);    // sets the digital pin 10 as input
  pinMode(joy2button, INPUT);    // sets the digital pin 11 as input
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  /*Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }*/

  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

//------------------------------------------------------------------------------

int thrPos, yawPos, pitPos, rolPos, lPush, rPush; // mapped joystick values centered at 3
uint8_t data[7];
uint8_t len = 3;

char key_video = "V";
char key_sensor = "S";
char key_ack = "A";
char hellosend[3] = "t\n";

bool ack = false;
bool saidhello = false;
int silence_count = 5;

void hello() {  // comm initiator block
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t expected[3] = "r\n";
  char hellosend[3] = "t\n";

  while (!saidhello) {
    // send to drone
    rf95.send((uint8_t*)hellosend, 3);
    rf95.waitPacketSent();
    Serial.println("sent hello");

    // receive from drone
    if (rf95.waitAvailableTimeout(1000)){
      if (rf95.recv(buf, &len)) {
        if (buf[0] == expected[0] && buf[1] == expected[1]) {
          Serial.println("received hello reply from drone");
          
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
      Serial.println("NO RESPONSE from drone");
    }
  }
}

void loop() {
  delay(10); // -----------------------------------------CHANGE BACK TO 10
  if (!saidhello)
    hello();
  
  thrPos = getValue(joy1Y);
  yawPos = getValue(joy1X);
  pitPos = getValue(joy2Y);
  rolPos = getValue(joy2X);
  lPush = 1 - digitalRead(joy1button);
  rPush = 1 - digitalRead(joy2button);
  
  Serial.print("Int variable values: ");
  Serial.print(thrPos);
  Serial.print(yawPos);
  Serial.print(pitPos);
  Serial.print(rolPos);
  Serial.print(lPush);
  Serial.println(rPush);
  
  createControlPacket();
  sendControlPacket();
  
  while (!ack) {
    receivePacket();
    if (silence_count >= 5) {
      saidhello = false;
      break;
    }
  }
}

int getValue(int pin) {
  int axisValue = analogRead(pin);                // swapped for sideways oreintation on breadboard
  //int axisMap = map(axisValue,0,1023,0,6);           // 10 bit mapped to a range of -3 to 3

  if (deadzone(axisValue)) {                    // ensures values of 0 when joystick is centered
    axisValue = 512;
  }
  return axisValue;
}

bool deadzone(int value) {
  if (value > 480 && value < 525) {           // ADJUST AFTER SOLDERING
    return true;
  }
  return false;
}

void createControlPacket() { // CHANGE TO 10 BIT
  yawPos = 1024 - yawPos;
  pitPos = 1024 - pitPos;
  
  data[0] = 0x43;   // control packet key "C" in ASCII
  data[1] = 0x00;   // reset each data byte
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0xFF;

  data[1] = (lPush<<4 | ((thrPos & 0x03C0)>>6));  // shift and pack button,
  data[2] = ((thrPos & 0x3F)<<2 | ((yawPos & 0x0300)>>8));  // x-axis, & y-axis values
  data[3] = (yawPos & 0xFF);

  data[4] = (rPush<<4 | ((pitPos & 0x03C0)>>6));  // shift and pack button,
  data[5] = ((pitPos & 0x3F)<<2 | ((rolPos & 0x0300)>>8));  // x-axis, & y-axis values
  data[6] = (rolPos & 0xFF);

  //Serial.println(data[0], BIN);
  //Serial.println(data[1], BIN);
  //Serial.println(data[2], BIN);
  //Serial.println(data[3], BIN);


  
}

void sendControlPacket() {
  rf95.send(data, 7);//sizeof(data));
  rf95.waitPacketSent();
  //Serial.print("Packet sent: ");
  //Serial.println((char*)data);
  ack = false;
}

void receivePacket() {
  if (rf95.waitAvailableTimeout(100)) {
    if (rf95.recv(data, &len)) {
      //Serial.print("Packet received: ");
      //Serial.println((char*)data);
      
      if (data[0] == key_video) {
        //Serial.println("Received a video packet");
        silence_count = 0;
        decodeVideoPacket();
      }

      if (data[0] == key_sensor) {
        //Serial.println("Received a sensor packet");
        silence_count = 0;
        decodeSensorPacket();
      }

      if (data[0] == key_ack) {
        //Serial.println("Received an acknowledgement packet");
        silence_count = 0;
        ack = true;
      }
    }
  }
  else {
    silence_count++;
    Serial.print("Radio silence from drone: ");
    Serial.println(silence_count);
  }
}

void decodeSensorPacket() {
  /*Sensor.x =   (data[1] & 0x40) >> 6;
  Sensor.y = ((data[1] & 0x38) >> 3) - 3;
  Sensor.z =  (data[1] & 0x07) - 3;

  Sensor.x =   (data[2] & 0x40) >> 6;
  Sensor.y = ((data[2] & 0x38) >> 3) - 3;
  Sensor.z =  (data[2] & 0x07) - 3;*/
}

void decodeVideoPacket() {
  /*Sensor.x =   (data[1] & 0x40) >> 6;
  Sensor.y = ((data[1] & 0x38) >> 3) - 3;
  Sensor.z =  (data[1] & 0x07) - 3;

  Sensor.x =   (data[2] & 0x40) >> 6;
  Sensor.y = ((data[2] & 0x38) >> 3) - 3;
  Sensor.z =  (data[2] & 0x07) - 3;*/
}
