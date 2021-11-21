#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

class pressureSense {
  private:
    Adafruit_BMP280 bmp; // use I2C interface
  private:
    void getpressure();
    void getaltitude();
    void setground();
  public:
    pressureSense();
    bool mBegin();
    void updatevalues();
    float aboveground();
  public:
    float pressure;
    float altitude;
    float ground;
};
