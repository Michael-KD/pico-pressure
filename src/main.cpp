#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_u-blox_GNSS_v3.h"
#include <MicroNMEA.h>

#include <pressure.h>

uint32_t OSR = 256; // Set the oversampling rate, options: 256, 512, 1024, 2048, 4096
uint16_t model = 1; // Set the model of the sensor, options: 1, 2
uint8_t address = 0x76; // Set the I2C address of the sensor: 0x76, 0x77

MS_5803 pressureSensor(OSR, address, model);
SFE_UBLOX_GNSS myGNSS;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));




unsigned long lastGPSReadTime = 0;
unsigned long lastPressureReadTime = 0;
long lastTime = 0;

float pressure, temperature;

struct GPSTimeData {
    uint32_t epoch;
    uint32_t micros;
    bool timeFullyResolved;
    bool timeValid;
    bool confirmedTime;
    uint8_t SIV;
    uint32_t localTimer;
};

GPSTimeData getGPSTimeData(SFE_UBLOX_GNSS &myGNSS) {
    GPSTimeData data;

    // Get Unix Epoch and microseconds
    data.epoch = myGNSS.getUnixEpoch(data.micros);

    // Get time resolution and validity
    data.timeFullyResolved = myGNSS.getTimeFullyResolved();
    data.timeValid = myGNSS.getTimeValid();
    data.confirmedTime = myGNSS.getConfirmedTime();

    // Get the number of satellites in view (SIV)
    data.SIV = myGNSS.getSIV();

    // Get the local time
    data.localTimer = millis();

    return data;
}



void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect.
    }

    Wire.begin();

    // Initialize pressure sensor
    if (!pressureSensor.begin()) {
        Serial.println("Failed to initialize MS5803 sensor!");
        while (1) ; // Halt execution
    }
    Serial.println("MS5803 sensor initialized successfully.");

    // Initialize GNSS module
    if (myGNSS.begin() == false) {
        Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA messages to processNMEA

    // myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA); // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it
    myGNSS.setNMEAOutputPort(Serial);
}

void loop() {
    unsigned long currentTime = millis();

    // if (currentTime - lastGPSReadTime >= 10000) {
    //     myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
    //     Serial.println("Checking GNSS");
    //     if(nmea.isValid() == true) {
    //         long latitude_mdeg = nmea.getLatitude();
    //         long longitude_mdeg = nmea.getLongitude();

    //         Serial.print("Latitude (deg): ");
    //         Serial.println(latitude_mdeg / 1000000., 6);
    //         Serial.print("Longitude (deg): ");
    //         Serial.println(longitude_mdeg / 1000000., 6);

    //         nmea.clear(); // Clear the MicroNMEA storage to make sure we are getting fresh data
    //     }
        



    //     lastGPSReadTime = currentTime;
    // }


    if (currentTime - lastPressureReadTime >= 30) {
        pressureSensor.readSensor();

        pressure = pressureSensor.getPressure();
        temperature = pressureSensor.getTemp();
        // Serial.print("Pressure: ");
        // Serial.print(pressure, 2);
        // Serial.print(" mbar, Temperature: ");
        // Serial.print(temperature, 2);
        // Serial.println(" C");

        // teleplot data
        Serial.print(">pressure:");
        Serial.println(pressure);
        Serial.print(">temperature:");
        Serial.println(temperature);


        lastPressureReadTime = currentTime;
    }

}