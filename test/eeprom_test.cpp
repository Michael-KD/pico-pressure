// EEPROM module
// On I2C1 bus, SDA is GPIO 6, SCL is GPIO 7
// AT24C32D EEPROM module, I2C address 0x50

#include <Arduino.h>
#include <Wire.h>

#define EEPROM_ADDRESS 0x50

void clearSerialInput() {
  uint32_t m = micros();
  while (micros() - m < 10000) {
    if (Serial.read() >= 0) {
      m = micros();
    }
  }
}

void writeToEEPROM(uint16_t address, uint8_t data) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((address >> 8) & 0xFF); // High byte of address
  Wire.write(address & 0xFF);        // Low byte of address
  Wire.write(data);                  // Data to write
  Wire.endTransmission();
  delay(5); // EEPROM write delay
  Serial.println("Data written to EEPROM.");
}

uint8_t readFromEEPROM(uint16_t address) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((address >> 8) & 0xFF); // High byte of address
  Wire.write(address & 0xFF);        // Low byte of address
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDRESS, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    Serial.println("Failed to read from EEPROM.");
    return 0xFF; // Return 0xFF if read fails
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Wire.begin();
  Serial.println("EEPROM test started.");
  Serial.println("Enter 1 to write data, or 2 to read data:");
}

void loop() {
  clearSerialInput();

  Serial.println("\nWaiting for input...");
  while (!Serial.available()) {
    yield();
  }

  char input = Serial.read();

  switch (input) {
    case '1': {
      Serial.println("Enter address (0-4095):");
      while (!Serial.available());
      uint16_t address = Serial.parseInt();
      if (address > 4095) {
        Serial.println("Invalid address. Must be between 0 and 4095.");
        return;
      }

      Serial.println("Enter data (0-255):");
      while (!Serial.available());
      uint8_t data = Serial.parseInt();
      if (data > 255) {
        Serial.println("Invalid data. Must be between 0 and 255.");
        return;
      }

      writeToEEPROM(address, data);
      break;
    }
    case '2': {
      Serial.println("Enter address (0-4095):");
      while (!Serial.available());
      uint16_t address = Serial.parseInt();
      if (address > 4095) {
        Serial.println("Invalid address. Must be between 0 and 4095.");
        return;
      }

      uint8_t data = readFromEEPROM(address);
      Serial.print("Data at address ");
      Serial.print(address);
      Serial.print(": ");
      Serial.println(data);
      break;
    }
    default:
      Serial.println("Invalid input. Enter 1 or 2:");
      return;
  }

  // Prompt the user again
  Serial.println("Enter 1 to write data, or 2 to read data:");
}