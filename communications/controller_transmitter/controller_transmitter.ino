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
#define joy1button 11 // left button
#define joy2button 10 // right button

void setup() 
{
  pinMode(joy1button, INPUT);    // sets the digital pin 10 as input
  pinMode(joy2button, INPUT);    // sets the digital pin 11 as input
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

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

int thrPos, yawPos, pitPos, rolPos, lPush, rPush; // mapped joystick values centered at 0
char radiopacket[20] = "               "; // char array layout for packet

int16_t packetnum = 0;  // packet counter, we increment per xmission
bool saidhello = false;

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
  delay(100);

  if (!saidhello)
    hello();
  packetnum++;
  //Serial.println(packetnum);
  if (packetnum >= 10) {
    saidhello = false;
    packetnum = 0;
  }
  
  thrPos = getValue(joy1X);
  yawPos = getValue(joy1Y);
  pitPos = getValue(joy2X);
  rolPos = getValue(joy2Y);
  lPush = 1 - digitalRead(joy1button);
  rPush = 1 - digitalRead(joy2button);
  createPacket();
  sendPacket();
}

int getValue(int pin) {
  int axisValue = analogRead(pin);                // swapped for sideways oreintation on breadboard
  int axisMap = map(axisValue,0,1023,-3,3);           // 10 bit mapped to a range of -4 to 4

  if (deadzone(axisValue)) {                    // ensures values of 0 when joystick is centered
    axisMap = 0;
  }
  return axisMap;
}

bool deadzone(int value) {
  if (value > 480 && value < 525) {
    return true;
  }
  return false;
}

void createPacket() {
  char thrData[2], yawData[2], pitData[2], rolData[2], lPushData[1], rPushData[1];                  // char arrays holding the mapped values as strings

  itoa(thrPos, thrData, 10);                    // convert integers to null-terminated string stored in arrays
  itoa(yawPos, yawData, 10);
  itoa(pitPos, pitData, 10);
  itoa(rolPos, rolData, 10);
  itoa(lPush, lPushData, 10);
  itoa(rPush, rPushData, 10);

  radiopacket[0]=' ';
  radiopacket[1]=thrData[0];                  // copies first element of array to packet array
  if (thrPos < 0) {                           // if negative, first element contains (-), so second element
    radiopacket[0]=thrData[0];
    radiopacket[1]=thrData[1];                // must also be copied
  }
  radiopacket[2]=' ';
  radiopacket[3]=yawData[0];
  if (yawPos < 0) {
    radiopacket[2]=yawData[0];
    radiopacket[3]=yawData[1];
  }
  radiopacket[4]=' ';
  radiopacket[5]=pitData[0];
  if (pitPos < 0) {
    radiopacket[4]=pitData[0];
    radiopacket[5]=pitData[1];
  }
  radiopacket[6]=' ';
  radiopacket[7]=rolData[0];
  if (rolPos < 0) {
    radiopacket[6]=rolData[0];
    radiopacket[7]=rolData[1];
  }
  radiopacket[8]=lPushData[0];
  radiopacket[9]=rPushData[0];
}

void sendPacket() {
  rf95.send((uint8_t *)radiopacket, 20);
  rf95.waitPacketSent();
  Serial.print("Packet sent: ");
  Serial.println(radiopacket);
}
