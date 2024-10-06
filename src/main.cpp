#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define DEBUG 0
#define SD_CS 17
#define SIZE 50
#define OSR 512 // Set the oversampling rate, options: 256, 512, 1024, 2048, 4096

unsigned long currentTime;
float pressure, temperature;

#include <MS5803_01.h> 
MS_5803 sensor = MS_5803(OSR);
int32_t pressureArray[SIZE];
int32_t temperatureArray[SIZE];
int arrayIndex = 0;
unsigned long lastLogTime = 0;
unsigned long timeArray[SIZE];

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
    String extension = ".csv";
    filename = baseFilename + extension;
    int fileIndex = 1;

    while (SD.exists(filename.c_str())) {
        filename = baseFilename + String(fileIndex) + extension;
        fileIndex++;
    }

    // write header
    File dataFile = SD.open(filename.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile.println("Time,Pressure,Temp");
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
        
        currentTime = millis();
        pressure = sensor.pressure();
        temperature = sensor.temperature();
        
        // teleplot
        if (DEBUG) {
            Serial.print(">pressure:");
            Serial.println(pressure);
            Serial.print(">temperature:");
            Serial.println(temperature);
        }

        if (arrayIndex == SIZE) {
            arrayIndex = 0;
            //log
            File dataFile = SD.open(filename.c_str(), FILE_WRITE);
            if (dataFile) {
                for (int i = 0; i < 50; i++) {
                    dataFile.print(timeArray[i]);
                    dataFile.print(",");
                    dataFile.print(pressureArray[i]);
                    dataFile.print(",");
                    dataFile.println(temperatureArray[i]);
                }
                dataFile.close();
            } else {
                if (DEBUG)
                Serial.println("Error logging to datalog");
            }
        }

        timeArray[arrayIndex] = currentTime - lastLogTime;
        pressureArray[arrayIndex] = (int32_t) round(pressure * 100);
        temperatureArray[arrayIndex] = (int32_t) round(temperature * 100);
        arrayIndex++;

        lastLogTime = currentTime;

        // lastPressureReadTime = currentTime;
    // }

}