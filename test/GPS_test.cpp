// GPS module
// On I2C1 bus, SDA is GPIO 6, SCL is GPIO 7
// MAX M10S module, I2C address 0x42


#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <MicroNMEA.h>

#define GPS_ADDRESS 0x42
#define GPS_EN_PIN 13


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

    data.epoch = myGNSS.getUnixEpoch(data.micros);
    data.timeFullyResolved = myGNSS.getTimeFullyResolved();
    data.timeValid = myGNSS.getTimeValid();
    data.confirmedTime = myGNSS.getConfirmedTime();
    data.SIV = myGNSS.getSIV();

    data.localTimer = millis();

    return data;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);

    pinMode(GPS_EN_PIN, OUTPUT);
    digitalWrite(GPS_EN_PIN, HIGH);
    delay(500);

    Wire1.setSDA(6);
    Wire1.setSCL(7);
    Wire1.setClock(400000);
    delay(100);
    Wire1.begin();
    // Wire.begin();
    
    delay(500);

    if (myGNSS.begin(Wire1) == false) {
        Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }
    Serial.println(F("u-blox GNSS module detected."));

    myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA messages to processNMEA

    // myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA); // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it
    myGNSS.setNMEAOutputPort(Serial);
}

void loop() {
    Serial.println("Checking GNSS");

    myGNSS.checkUblox(); // See if new data is available. Process bytes as they come in.
    delay(500);
        // if(nmea.isValid() == true) {
        long latitude_mdeg = nmea.getLatitude();
        long longitude_mdeg = nmea.getLongitude();

        Serial.print("Latitude (deg): ");
        Serial.println(latitude_mdeg / 1000000., 6);
        Serial.print("Longitude (deg): ");
        Serial.println(longitude_mdeg / 1000000., 6);

        nmea.clear();
        // }
        
}