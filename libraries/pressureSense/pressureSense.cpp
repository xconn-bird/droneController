#include "pressureSense.h"

pressureSense::pressureSense() {
  this->pressure = 0;
  this->altitude = 0;
}

bool pressureSense::mBegin() {
  if(!bmp.begin()) {
    return false;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  updatevalues();
  setground();
  return true;
}

void pressureSense::getpressure() {
  this->pressure = bmp.readPressure();
}

void pressureSense::getaltitude() {
  this->altitude = bmp.readAltitude();
}

void pressureSense::updatevalues() {
  getpressure();
  getaltitude();
}

void pressureSense::setground() {
  this->ground = this->altitude;
}

float pressureSense::aboveground() {
  return this->altitude - this->ground;
}
