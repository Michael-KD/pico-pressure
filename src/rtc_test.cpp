// RTC module
// On I2C1 bus, SDA is GPIO 6, SCL is GPIO 7
// DS3231SN RTC module, I2C address 0x68

#include <Arduino.h>
#include <Wire.h>

#define DS3231_ADDRESS 0x68

void clearSerialInput() {
  uint32_t m = micros();
  while (micros() - m < 10000) {
    if (Serial.read() >= 0) {
      m = micros();
    }
  }
}

void writeTimeToRTC() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x00); // Set pointer to seconds register

  // Example: Set time to 12:34:56 on 7th May 2025 (Tuesday)
  Wire.write(0x56); // Seconds
  Wire.write(0x34); // Minutes
  Wire.write(0x12); // Hours (24-hour format)
  Wire.write(0x02); // Day of the week (1 = Sunday, 2 = Monday, etc.)
  Wire.write(0x07); // Date
  Wire.write(0x05); // Month
  Wire.write(0x25); // Year (last two digits of 2025)

  Wire.endTransmission();
  Serial.println("Time written to RTC.");
}

void readTimeFromRTC() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x00); // Set pointer to seconds register
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, 7);
  if (Wire.available() == 7) {
    uint8_t seconds = Wire.read();
    uint8_t minutes = Wire.read();
    uint8_t hours = Wire.read();
    uint8_t dayOfWeek = Wire.read();
    uint8_t date = Wire.read();
    uint8_t month = Wire.read();
    uint8_t year = Wire.read();

    Serial.print("Time: ");
    Serial.print(hours, HEX);
    Serial.print(":");
    Serial.print(minutes, HEX);
    Serial.print(":");
    Serial.println(seconds, HEX);

    Serial.print("Date: ");
    Serial.print(date, HEX);
    Serial.print("/");
    Serial.print(month, HEX);
    Serial.print("/20");
    Serial.println(year, HEX);

    Serial.print("Day of the week: ");
    Serial.println(dayOfWeek);
  } else {
    Serial.println("Failed to read time from RTC.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Wire.begin();
  Serial.println("RTC test started.");
  Serial.println("Enter 1 to write current time, or 2 to read current time:");
}

void loop() {
  clearSerialInput();

  Serial.println("\nWaiting for input...");
  while (!Serial.available()) {
    yield();
  }

  char input = Serial.read();

  switch (input) {
    case '1':
      Serial.println("Writing current time to RTC...");
      writeTimeToRTC();
      break;
    case '2':
      Serial.println("Reading current time from RTC...");
      readTimeFromRTC();
      break;
    default:
      Serial.println("Invalid input. Enter 1 or 2:");
      return;
  }

  // Prompt the user again
  Serial.println("Enter 1 to write current time, or 2 to read current time:");
}