#include <SPI.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define joy1X A0 // throttle
#define joy1Y A1 // yaw
#define joy2X A2 // pitch
#define joy2Y A3 // roll
#define joy1button 11 // left button
#define joy2button 10 // right button

int thrPos, yawPos, pitPos, rolPos, lPush, rPush; // mapped joystick values centered at 0
char radiopacket[20] = "               "; // char array layout for packet

void setup() 
{
  pinMode(joy1button, INPUT);    // sets the digital pin 10 as input
  pinMode(joy2button, INPUT);    // sets the digital pin 11 as input
  
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
}

void loop() {
  delay(10);
  thrPos = getValue(joy1X);
  yawPos = getValue(joy1Y);
  pitPos = getValue(joy2X);
  rolPos = getValue(joy2Y);
  lPush = digitalRead(joy1button);
  rPush = digitalRead(joy2button);
  
  createPacket();

  Serial.print("Sending: ");
  Serial.println(radiopacket);
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
