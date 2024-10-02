#include "pressure.h"
#include "2cb.h"
#include <Wire.h>
#include <Arduino.h>

#define CMD_RESET		0x1E	// ADC reset command
#define CMD_ADC_READ	0x00	// ADC read command
#define CMD_ADC_CONV	0x40	// ADC conversion command
#define CMD_ADC_D1		0x00	// ADC D1 conversion
#define CMD_ADC_D2		0x10	// ADC D2 conversion
#define CMD_ADC_256		0x00	// ADC resolution=256
#define CMD_ADC_512		0x02	// ADC resolution=512
#define CMD_ADC_1024	0x04	// ADC resolution=1024
#define CMD_ADC_2048	0x06	// ADC resolution=2048
#define CMD_ADC_4096	0x08	// ADC resolution=4096

MS_5803::MS_5803(uint32_t OSR, uint16_t address, uint8_t model) {
    _OSR = OSR;
    _address = address;
    _model = model;
    _delay = 10;
}

bool MS_5803::begin() {
    Wire.beginTransmission(_address);
    if (Wire.endTransmission() != 0) {
        return false;  // Sensor not found
    }
    calibrate();
    return true;
}


void MS_5803::calibrate() {
    Wire.beginTransmission(_address); // Start communication with device at address
    Wire.write(CMD_RESET);            // Send the byte 0x1E
    Wire.endTransmission();           // End the transmission

    delay(10);

    _C1 = read16(_address, 0xA2);
    _C2 = read16(_address, 0xA4);
    _C3 = read16(_address, 0xA6);
    _C4 = read16(_address, 0xA8);
    _C5 = read16(_address, 0xAA);
    _C6 = read16(_address, 0xAC);
}

void MS_5803::readSensor() {
    uint32_t D1, D2;
    int32_t dT, TEMP;
    int64_t OFF, SENS;
    int32_t T2 = 0;
    int64_t OFF2 = 0, SENS2 = 0;



    // MS5803_01BA address, 0x76(118)
    // 0x40(64) Pressure conversion(OSR = INPUT) command
    Wire.beginTransmission(_address);
    Wire.write((uint8_t)(CMD_ADC_CONV | _OSR | CMD_ADC_D1));
    Wire.endTransmission();

    delay(_delay);

    // Read digital pressure value
    // Read data back from 0x00(0), 3 bytes
    // D1 MSB2, D1 MSB1, D1 LSB
    D1 = read24(_address, CMD_ADC_READ);

    // MS5803_01BA address, 0x76(118)
    // 0x50(64) Temperature conversion(OSR = INPUT) command
    Wire.beginTransmission(_address);
    Wire.write((uint8_t)(CMD_ADC_CONV | _OSR | CMD_ADC_D2));
    Wire.endTransmission();

    delay(_delay);

    // Read digital temperature value
    // Read data back from 0x00(0), 3 bytes
    // D2 MSB2, D2 MSB1, D2 LSB
    D2 = read24(_address, CMD_ADC_READ);

    dT = D2 - (uint32_t)_C5 * 256;
    TEMP = 2000 + ((int64_t)dT * _C6) / 8388608;

    OFF = 0;
    SENS = 0;

    if (_model == 1) {
        OFF = (int64_t)_C2 * 65536 + ((int64_t)_C4 * dT) / 128;
        SENS = (int64_t)_C1 * 32768 + ((int64_t)_C3 * dT) / 256;

        if (TEMP >= 2000) {
            T2 = 0;
            OFF2 = 0;
            SENS2 = 0;
            if (TEMP > 4500) {
                SENS2 = SENS2 - ((TEMP - 4500) * (TEMP - 4500)) / 8;
            }
        } else {
            T2 = ((int64_t)dT * dT) / 2147483648;
            OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000));
            SENS2 = 7 * ((TEMP - 2000) * (TEMP - 2000)) / 8;
            if (TEMP < -1500) {
                SENS2 = SENS2 + 2 * ((TEMP + 1500) * (TEMP + 1500));
            }
        }
    }
    // } else if (_model == 2) {
    //     OFF = (int64_t)_C2 * 131072 + ((int64_t)_C4 * dT) / 64;
    //     SENS = (int64_t)_C1 * 65536 + ((int64_t)_C3 * dT) / 128;

    //     if (TEMP >= 2000) {
    //         T2 = 0;
    //         OFF2 = 0;
    //         SENS2 = 0;
    //     } else {
    //         T2 = ((int64_t)dT * dT) / 2147483648;
    //         OFF2 = 61 * ((TEMP - 2000) * (TEMP - 2000)) / 16;
    //         SENS2 = 2 * ((TEMP - 2000) * (TEMP - 2000));
    //         if (TEMP < -1500) {
    //             OFF2 = OFF2 + 20 * ((TEMP + 1500) * (TEMP + 1500));
    //             SENS2 = SENS2 + 12 * ((TEMP + 1500) * (TEMP + 1500));
    //         }
    //     }
    // }
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    _rawPressure = (((int64_t)D1 * SENS) / 2097152 - OFF) / 32768;
    _pressure = (float)_rawPressure / 100.0;
    // Serial.println(_rawPressure);
    // Serial.println(_pressure);

    _cTemp = (float)TEMP / 100.0;
    // Serial.println(TEMP);
    // Serial.println(_cTemp);

}

float MS_5803::getPressure() {
    return _pressure;
}
float MS_5803::getTemp() {
    return _cTemp;
}

uint16_t MS_5803::getC(int i) {
    switch (i) {
        case 1:
            return _C1;
        case 2:
            return _C2;
        case 3:
            return _C3;
        case 4:
            return _C4;
        case 5:
            return _C5;
        case 6:
            return _C6;
        default:
            return 0;
    }
}