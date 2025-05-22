#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include "MS5803_spi.h"

#include "pico/multicore.h"
#include "hardware/sync.h"



// Pressure sensor 
#define OSR 256
#define PRESSURE_CS_PIN 9
MS_5803 pressureSensor(OSR, PRESSURE_CS_PIN, 1); 

// SD card
#define SD_CS_PIN 17
#define LOG_FILENAME "data.csv"

SdFs sd;
FsFile logFile;

float startTime = 0;

struct PressureSample {
    float pressure;
    float temperature;
    float timestamp;
};

const size_t BUFFER_SIZE = 256;
volatile PressureSample buffer[BUFFER_SIZE];
volatile size_t head = 0; // write index (loop)
volatile size_t tail = 0; // read index (loop1)

size_t droppedSamples = 0;

// *** CORE 0 ****

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(200);

    Serial.println("Pressure test started.");
    pressureSensor.initializeMS_5803();
    Serial.println("Pressure sensor initialized.");
    delay(200);

    droppedSamples = 0;
    // addedSamples = 0;

    // multicore_launch_core1(setup1);
    startTime = micros();    

}


void loop() {
    pressureSensor.readSensor();
    float timestamp = micros() - startTime;
    float pressure = pressureSensor.pressure();
    float temperature = pressureSensor.temperature();

    // Calculate next head index
    size_t nextHead = (head + 1) % BUFFER_SIZE;

    // Check for buffer full (don't overwrite unread data)
    if (nextHead != tail) {
        // Store data in buffer
        buffer[head].pressure = pressure;
        buffer[head].temperature = temperature;
        buffer[head].timestamp = timestamp;
        head = nextHead;
        // addedSamples++;
        // Serial.print("Added samples: ");
        // Serial.println(addedSamples);

    } else {
        // Buffer is full, increment dropped samples
        droppedSamples++;
        Serial.print("Dropped samples: ");
        Serial.println(droppedSamples);
    }

}

// *** CORE 1 ****

void setup1() {
    while (!Serial);
    delay(200);

    if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
        Serial.println("SD init failed!");
        while (1);
    }
    logFile = sd.open(LOG_FILENAME, O_WRONLY | O_CREAT | O_AT_END);
    if (!logFile) {
        Serial.println("File open failed!");
        while (1);
    }
    // Write CSV header if file is new
    if (logFile.size() == 0) {
        logFile.println("micros,pressure,temperature");
        logFile.flush();
    }

}


void loop1() {
    static int writeCount = 0;
    // Check if buffer is not empty
    while (tail != head) {
        // Copy out the sample
        PressureSample sample;
        sample.pressure = buffer[tail].pressure;
        sample.temperature = buffer[tail].temperature;
        sample.timestamp = buffer[tail].timestamp;

        // Advance tail
        tail = (tail + 1) % BUFFER_SIZE;

        // Log to SD card
        logFile.print(sample.timestamp);
        logFile.print(",");
        logFile.print(sample.pressure, 4);
        logFile.print(",");
        logFile.println(sample.temperature, 2);
        writeCount++;

        if (writeCount >= 100) {
            logFile.flush();
            writeCount = 0;
        }
    }
    delay(1); // Optional
}