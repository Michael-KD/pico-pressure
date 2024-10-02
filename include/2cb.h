#ifndef __2CB__
#define __2CB__

// i2c wrapper functions

#include <Arduino.h>
#include <Wire.h>

void readI2CBlockData(uint8_t address, uint8_t reg, uint8_t* data, uint8_t numBytes);
uint16_t bytesToUint16(uint8_t* bytes);
uint16_t read16(uint8_t address, uint8_t reg);
uint32_t read24(uint8_t address, uint8_t reg);
uint32_t read32(uint8_t address, uint8_t reg);


#endif