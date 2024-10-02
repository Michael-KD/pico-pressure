#include "2cb.h" // i2c wrapper functions

void readI2CBlockData(uint8_t address, uint8_t reg, uint8_t* data, uint8_t numBytes) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();  // Send repeated start
    Wire.requestFrom(address, numBytes);

    for (int i = 0; i < numBytes && Wire.available(); i++) {
        data[i] = Wire.read();
    }
}

uint16_t read16(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(address, (uint8_t)2);
    uint16_t value = Wire.read() << 8 | Wire.read();
    
    return value;
}

uint32_t read24(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(address, (uint8_t)3);
    uint32_t value = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
    
    return value;
}

uint32_t read32(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(address, (uint8_t)4);
    uint32_t value = Wire.read() << 24 | Wire.read() << 16 | Wire.read() << 8 | Wire.read();
    
    return value;
}

