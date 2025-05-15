// Pressure sensor
// On I2C0 bus, SDA is GPIO 4, SCL is GPIO 5
// MS5803 module, I2C address 0x76

#include <Arduino.h>
#include <Wire.h>
// #include "MS5803.h"
#include "MS5803_spi.h"

// #define PRESSURE_ADDRESS 0x76
#define OSR 256
#define CS_PIN 9

// MS_5803 pressureSensor(OSR, PRESSURE_ADDRESS, 1);
MS_5803 pressureSensor(OSR, CS_PIN, 1); 


void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);


    // Wire.setClock(400000);
    // Wire.begin();

    Serial.println("Pressure test started.");
    pressureSensor.initializeMS_5803();
    Serial.println("Pressure sensor initialized.");
    delay(500);
    
}

void loop() {
    static unsigned long lastTime = 0;
    static unsigned long startTime = 0;
    static unsigned long totalDelay = 0;
    static int readingCount = 0;

    unsigned long now = millis();

    unsigned long t0 = micros();
    pressureSensor.readSensor();
    unsigned long t1 = micros();
    float pressure = pressureSensor.pressure();
    float temperature = pressureSensor.temperature();
    unsigned long t2 = micros();

    Serial.print(">pressure:");
    Serial.println(pressure);
    Serial.print(">temperature:");
    Serial.println(temperature);
    unsigned long t3 = micros();

    if (readingCount == 0) {
        startTime = now;
    } else {
        totalDelay += (now - lastTime);
    }
    lastTime = now;
    readingCount++;

    if (readingCount == 100) {
        unsigned long elapsed = now - startTime;
        float avgDelay = totalDelay / 99.0; // 99 intervals between 100 readings
        float freq = 1000.0 / avgDelay;
        Serial.print(">frequency (hz):");
        Serial.println(freq, 2);
        Serial.print(">delay (ms):");
        Serial.println(avgDelay, 2);
        readingCount = 0;
        totalDelay = 0;
    }
    unsigned long t4 = micros();

    // Serial.print(">readSensor time (us): ");
    // Serial.println(t1 - t0);
    // Serial.print(">conversion time (us): ");
    // Serial.println(t2 - t1);
    // Serial.print(">print frequency time (us): ");
    // Serial.println(t4 - t3);
    // unsigned long t5 = micros();

    // Serial.print(">print time (us): ");
    // Serial.println(t5 - t4);

}