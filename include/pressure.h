#ifndef __pressure__
#define __pressure__

#include <Arduino.h>
#include <map>
#include <utility>


class MS_5803 {
public:
	// Constructor for the class. 
	// The argument is the desired oversampling resolution, which has 
	// values of 256, 512, 1024, 2048, 4096
	// Second argument is the I2C address of the unit: either 0x76 or 0x77
    MS_5803(uint32_t OSR, uint16_t address, uint8_t model);
    bool begin();
    void calibrate();
    void readSensor();
    float getPressure();
    float getTemp();
    uint16_t getC(int i);


private:
    uint32_t _OSR;
    uint16_t _address;
    uint8_t _model;
    int32_t _rawPressure;
    float _pressure;
    float _cTemp;
    uint16_t _C1, _C2, _C3, _C4, _C5, _C6;
    double _delay;
};

#endif