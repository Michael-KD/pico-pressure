#include <Arduino.h>
#include <Wire.h>
#include <pressure.h>
#include <SD.h>

#define DEBUG 1
#define SD_CS 17

uint32_t OSR = 256; // Set the oversampling rate, options: 256, 512, 1024, 2048, 4096
uint16_t model = 1; // Set the model of the sensor, options: 1, 2
uint8_t address = 0x76; // Set the I2C address of the sensor: 0x76, 0x77

float pressure, temperature;
unsigned long lastPressureReadTime = 0;
MS_5803 pressureSensor(OSR, address, model);


void setup() {
    if (DEBUG) {
        Serial.begin(115200);
        while (!Serial) {
            ; // Wait for serial port to connect.
        }
        delay(1000);
        Serial.println("Serial connected.");
    }
    Wire.begin();

    // Initialize pressure sensor
    if (!pressureSensor.begin()) {
        if (DEBUG)
        Serial.println("Failed to initialize MS5803 sensor!");
        while (1) ; // Halt execution
    }
    if (DEBUG)
    Serial.println("MS5803 sensor initialized successfully.");

    // Initialize SD card
    if (!SD.begin(SD_CS)) {
        if (DEBUG)
        Serial.println("Failed to initialize SD card!");
        while (1) ; // Halt execution
    }
    if (DEBUG)
    Serial.println("SD card initialized successfully.");

    String filename = "datalog" + String(millis()) + ".txt";
    const char *filename_cstr = filename.c_str();

    // write header
    File dataFile = SD.open(filename_cstr, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Pressure, Temperature");
        dataFile.close();
    } else {
        if (DEBUG)
        Serial.println("Error opening datalog.txt");
    }    
}


void loop() {
    unsigned long currentTime = millis();

    if (currentTime - lastPressureReadTime >= 30) {
        pressureSensor.readSensor();

        pressure = pressureSensor.getPressure();
        temperature = pressureSensor.getTemp();

        // teleplot
        if (DEBUG) {
            Serial.print(">pressure:");
            Serial.println(pressure);
            Serial.print(">temperature:");
            Serial.println(temperature);
        }

        // sd card logging
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
            dataFile.print(pressure);
            dataFile.print(",");
            dataFile.println(temperature);
            dataFile.close();
        } else {
            if (DEBUG)
            Serial.println("Error opening datalog.txt");
        }


        lastPressureReadTime = currentTime;
    }

}