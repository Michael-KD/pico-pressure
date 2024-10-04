#include <Arduino.h>
#include <Wire.h>
// #include <pressure .h>
#include <SD.h>

#define DEBUG 1
#define SD_CS 17

float pressure, temperature;
unsigned long lastPressureReadTime = 0;

#include <MS5803_01.h> 
const int osr = 256; // Set the oversampling rate, options: 256, 512, 1024, 2048, 4096
MS_5803 sensor = MS_5803(osr);

String filename;

void setup() {
    if (DEBUG) {
        Serial.begin(115200);
        while (!Serial) {
            ; // Wait for serial port to connect.
        }
        delay(500);
        Serial.println("Serial connected.");
    }

    // Initialize pressure sensor
    if (sensor.initializeMS_5803()) {
        Serial.println("MS5803 CRC check OK.");
    } else {
        Serial.println("MS5803 CRC check FAILED!");
        // keep retrying until successful
        while (!sensor.initializeMS_5803()) {
            delay(500);
            Serial.println("Retrying MS5803 CRC check...");
        }
    }

    // Initialize SD card
    if (!SD.begin(SD_CS)) { // SD_CS or BUILTIN_SDCARD
        if (DEBUG)
        Serial.println("Failed to initialize SD card!");
        // keep retrying until successful
        while (!SD.begin(SD_CS)) {
            delay(500);
            if (DEBUG)
            Serial.println("Retrying SD card initialization...");
        }
    }
    if (DEBUG)
    Serial.println("SD card initialized successfully.");

    // Check if file exists and increment filename if necessary
    String baseFilename = "test";
    String extension = ".txt";
    filename = baseFilename + extension;
    int fileIndex = 1;

    while (SD.exists(filename.c_str())) {
        filename = baseFilename + String(fileIndex) + extension;
        fileIndex++;
    }

    // write header
    File dataFile = SD.open(filename.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile.println("Time, Pressure, Temperature");
        dataFile.close();
    } else {
        if (DEBUG)
        Serial.println("Error opening datalog for header");
    }    
}


void loop() {
    // unsigned long currentTime = millis();

    // if (currentTime - lastPressureReadTime >= 5) {
        sensor.readSensor();
        
        pressure = sensor.pressure();
        temperature = sensor.temperature();
        
        // teleplot
        if (DEBUG) {
            Serial.print(">pressure:");
            Serial.println(pressure);
            Serial.print(">temperature:");
            Serial.println(temperature);
        }

        // sd card logging
        File dataFile = SD.open(filename.c_str(), FILE_WRITE);
        if (dataFile) {
            dataFile.print(millis());
            dataFile.print(",");
            dataFile.print(pressure);
            dataFile.print(",");
            dataFile.println(temperature);
            dataFile.close();
        } else {
            if (DEBUG)
            Serial.println("Error logging to datalog");
        }


        // lastPressureReadTime = currentTime;
    // }

}