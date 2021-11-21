#include "pressureSense.h"

pressureSense alt;

void setup() {
  Serial.begin(115200);
  while(!Serial)
    delay(10);
  if (!alt.mBegin())
    while (1) delay(10);
}

void loop() {
  alt.updatevalues();

  Serial.print(F("TDistance above ground:"));
  Serial.print(alt.aboveground());
  Serial.println(" m");
}
