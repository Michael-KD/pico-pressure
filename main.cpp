#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"




void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("Serial monitor opened!");
}

void loop() {
  Serial.println("Hello World!");
  delay(1000);

  // SD Card test


  // UART header test


  // GPS test


  // RTC test

  // Wtatchdog test


  // 


  // Pressure sensor test


  // Dual core test


}