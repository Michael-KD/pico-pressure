#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define DEBUG 1
#define LOGGING 0
#define SD_CS 17
#define SIZE 100 // Number of entries to log before writing to SD card
#define OSR 512 // Set the oversampling rate, options: 256, 512, 1024, 2048, 4096
#define FILE_DURATION 28800000 // 8 hours in milliseconds

unsigned long currentTime;
float pressure, temperature;

#include <MS5803_01.h> 
MS_5803 sensor = MS_5803(OSR, 0x76, 1);

String filename;
String baseFilename = "data";
String extension = ".csv";
char fileIterator = 'a';
unsigned long fileCreationTime;

struct LogEntry {
    uint32_t time;
    int32_t pressure;
    int32_t temperature;
};

LogEntry logBuffer[SIZE];
int bufferIndex = 0;
unsigned long lastLogTime = 0;

String findAvailableFilename() {
    String newBaseFilename = baseFilename;
    int counter = 1;
    while (SD.exists((newBaseFilename + "_a" + extension).c_str())) {
        newBaseFilename = baseFilename + String(counter);
        counter++;
    }
    return newBaseFilename;
}

void createNewFile() {
    fileCreationTime = millis();
    if (fileIterator > 'z' || !SD.exists(filename.c_str())) {
        fileIterator = 'a';
        baseFilename = findAvailableFilename();
    }
    filename = baseFilename + "_" + String(fileIterator) + extension;
    fileIterator++;

    // write header
    File dataFile = SD.open(filename.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile.println("Time,Pressure,Temp");
        dataFile.close();
        if (DEBUG) Serial.println("Created new file: " + filename);
    } else {
        if (DEBUG) Serial.println("Error opening new datalog for header");
    }
}

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

    if (LOGGING) {
        // Initialize SD card
        if (!SD.begin(SD_CS)) {
            if (DEBUG) Serial.println("Failed to initialize SD card!");
            // keep retrying until successful
            while (!SD.begin(SD_CS)) {
                delay(500);
                if (DEBUG) Serial.println("Retrying SD card initialization...");
            }
        }
        if (DEBUG) Serial.println("SD card initialized successfully.");

        createNewFile();
    }
}

void loop() {
    sensor.readSensor();
    
    currentTime = millis();
    pressure = sensor.pressure();
    temperature = sensor.temperature();
    
    if (DEBUG) {
        Serial.print(">pressure:");
        Serial.println(pressure);
        Serial.print(">temperature:");
        Serial.println(temperature);
    }

    if (LOGGING) {
    
        logBuffer[bufferIndex] = {
            static_cast<uint32_t>(currentTime - lastLogTime),
            static_cast<int32_t>(round(pressure * 100)),
            static_cast<int32_t>(round(temperature * 100))
        };

        bufferIndex++;

        if (bufferIndex == SIZE) {
            bufferIndex = 0;
            //log
            File dataFile = SD.open(filename.c_str(), FILE_WRITE);
            if (dataFile) {
                for (int i = 0; i < SIZE; i++) {
                    dataFile.print(logBuffer[i].time);
                    dataFile.print(",");
                    dataFile.print(logBuffer[i].pressure);
                    dataFile.print(",");
                    dataFile.println(logBuffer[i].temperature);
                }
                dataFile.close();
            } else {
                if (DEBUG) Serial.println("Error logging to datalog");
            }
        }

        lastLogTime = currentTime;

        // Check if it's time to create a new file
        if (currentTime - fileCreationTime >= FILE_DURATION) {
            createNewFile();
        }
    }
}