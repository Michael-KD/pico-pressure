#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"



void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("Serial monitor opened!");
  Wire1.begin();
}

void loop() {
  Wire1.begin();
  Serial.println("Hello World!");
  delay(1000);

}