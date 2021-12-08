#include "Adafruit_OV7670.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

 int USB = 1;
 int video = 10;
 


void setup() {
  // put your setup code here, to run once:
     Serial.begin(9600);
     // Try to locate the camera
     if (cam.begin()) {
    Serial.println("Camera Found:");
        } 
    else {
    Serial.println("No camera found?");
    return;
  }

     

       Serial.println("starting video.");
        delay(3000);

        if (! cam.takePicture()) 
         Serial.println("Failed to snap!");
        else 
         Serial.println("Picture taken!");
         
         cam.resume();
       
       uint8_t imgsize = cam.getImageSize();
       Serial.print("Image size: ");
       if (imgsize == VC0706_640x480) Serial.println("640x480");
       if (imgsize == VC0706_320x240) Serial.println("320x240");
       if (imgsize == VC0706_160x120) Serial.println("160x120");

       
     
}
