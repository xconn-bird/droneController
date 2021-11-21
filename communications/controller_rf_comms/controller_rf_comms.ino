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

int thrVal, thrPos, yawPos, yawVal, pitPos, rolPos, lPush, rPush = 0; // mapped joystick values centered at 3
int yawGyro, pitGyro, rolGyro; // sensor packet values
uint8_t data[7];
uint8_t recv_data[7];
uint8_t len = 7;

uint8_t key_video[2] = "V";
uint8_t key_sensor[2] = "S";
uint8_t key_ack[2] = "A";
char hellosend[3] = "t\n";

bool ack = false;
bool saidhello = false;
int silence_count = 5;
int drop_count = 0;

float thrscaler = 0;
float thrvalue = 0;
float yawscaler = 0;
float yawvalue = 0;

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
  delay(50); // -----------------------------------------CHANGE BACK TO 10
  if (!saidhello)
    hello();
  
  thrPos = getValue(joy1Y);
  yawPos = getValue(joy1X);
  pitPos = getValue(joy2Y);
  rolPos = getValue(joy2X);
  lPush = 1 - digitalRead(joy1button);
  rPush = 1 - digitalRead(joy2button);
  
  createControlPacket();
  sendControlPacket();
  
  Serial.print("Inputs: thr= ");
  Serial.print(thrVal-105); Serial.print(",yaw= ");
  Serial.print(yawVal-362); Serial.print(",pit= ");
  Serial.print(pitPos-48); Serial.print(",rol= ");
  Serial.print(rolPos-48); Serial.print(", ");
  Serial.print(lPush); Serial.print(", ");
  Serial.print(rPush);
  
  Serial.print("  Gyro: yaw= ");
  Serial.print(yawGyro); Serial.print(",pit= ");
  Serial.print(pitGyro); Serial.print(",rol= ");
  Serial.println(rolGyro);
  
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
  thrPos = map(thrPos,0,1024,-100,101);
  yawPos = map(yawPos,0,1024,-359,360);
  pitPos = map(pitPos,0,1024,-45,46);
  rolPos = map(rolPos,0,1024,-45,46);

  throttleCalc();
  yawCalc();

  
  thrVal += 105;
  yawVal += 362;
  pitPos += 48;
  rolPos += 48;
  

  
  data[0] = 0x43;   // control packet key "C" in ASCII
  data[1] = 0x00;   // reset each data byte
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;

  data[1] = (lPush<<4 | ((thrVal & 0x03C0)>>6));  // shift and pack button,
  data[2] = ((thrVal & 0x3F)<<2 | ((yawVal & 0x0300)>>8));  // x-axis, & y-axis values
  data[3] = (yawVal & 0xFF);

  data[4] = (rPush<<4 | ((pitPos & 0x03C0)>>6));  // shift and pack button,
  data[5] = ((pitPos & 0x3F)<<2 | ((rolPos & 0x0300)>>8));  // x-axis, & y-axis values
  data[6] = (rolPos & 0xFF);





  
  //Serial.println(data[0], BIN);
  //Serial.println(data[1], BIN);
  //Serial.println(data[2], BIN);
  //Serial.println(data[3], BIN);
}

void throttleCalc() {
  thrscaler = 0.05 * thrPos;
  thrvalue = thrvalue + thrscaler;
    
  if (thrvalue < 0) {
    thrvalue = 0;
  }
  if (thrvalue > 100) {
    thrvalue = 100;
  }
  thrVal = thrvalue;
}

void yawCalc() {
  yawscaler = 0.05 * yawPos;
  yawvalue = yawvalue + yawscaler;
    
  if (yawvalue < -360) {
    yawvalue = -360;
  }
  if (yawvalue > 360) {
    yawvalue = 360;
  }
  yawVal = yawvalue;
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
    if (rf95.recv(recv_data, &len)) {
      //Serial.print("Packet received: ");
      //Serial.println((char*)recv_data);
      
      if (recv_data[0] == key_video[0]) {
        //Serial.println("Received a video packet");
        silence_count = 0;
        decodeVideoPacket();
      }

      if (recv_data[0] == key_sensor[0]) {
        //Serial.println("Received a sensor packet");
        silence_count = 0;
        decodeSensorPacket();
      }

      if (recv_data[0] == key_ack[0]) {
        //Serial.println("Received an acknowledgement packet");
        silence_count = 0;
        ack = true;
      }
    }
  }
  else {
    drop_count++;
    silence_count++;
    Serial.print("Radio silence from drone: ");
    Serial.println(silence_count);
    Serial.print("Total dropped packet count: ");
    Serial.println(drop_count);
  }
}

void decodeSensorPacket() {
  /*Serial.println(recv_data[0], BIN);
  Serial.println(recv_data[1], BIN);
  Serial.println(recv_data[2], BIN);
  Serial.println(recv_data[3], BIN);
  Serial.println(recv_data[4], BIN);
  Serial.println(recv_data[5], BIN);
  Serial.println(recv_data[6], BIN);*/
  yawGyro = ((recv_data[2] & 0x03) << 8) | (recv_data[3]);        // mask for bits 2-0
  pitGyro = ((recv_data[4] & 0x0F) << 6) | ((recv_data[5] & 0xFC) >> 2);  // mask for bits 5-3 and shift right
  rolGyro = ((recv_data[5] & 0x03) << 8) | (recv_data[6]);        // mask for bits 2-0

}

void decodeVideoPacket() {
  /*Sensor.x =   (recv_data[1] & 0x40) >> 6;
  Sensor.y = ((recv_data[1] & 0x38) >> 3) - 3;
  Sensor.z =  (recv_data[1] & 0x07) - 3;

  Sensor.x =   (recv_data[2] & 0x40) >> 6;
  Sensor.y = ((recv_data[2] & 0x38) >> 3) - 3;
  Sensor.z =  (recv_data[2] & 0x07) - 3;*/
}
