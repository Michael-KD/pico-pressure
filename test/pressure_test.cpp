// Pressure sensor
// On I2C0 bus, SDA is GPIO 4, SCL is GPIO 5
// MS5803 module, I2C address 0x76

#include <Arduino.h>
#include <Wire.h>
#include "MS5803_01.h"

#define PRESSURE_ADDRESS 0x76
#define OSR 512

MS_5803 pressureSensor(OSR, PRESSURE_ADDRESS, 1);


void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    Wire.begin();
    Serial.println("Pressure test started.");
    pressureSensor.initializeMS_5803();
    Serial.println("Pressure sensor initialized.");
    delay(1000);
}

void loop() {
    pressureSensor.readSensor();
    float pressure = pressureSensor.pressure();
    float temperature = pressureSensor.temperature();
    Serial.print(">pressure:");
    Serial.println(pressure);
    Serial.print(">temperature:");
    Serial.println(temperature);
}