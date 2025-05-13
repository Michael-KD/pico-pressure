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

void resetTimeToRTC() {
    Wire1.beginTransmission(DS3231_ADDRESS);
    Wire1.write(0x00); // Set pointer to seconds register

    // Set time to 00:00:00 on 1st Jan 2000 (Saturday, UTC epoch for DS3231)
    Wire1.write(0x00); // Seconds
    Wire1.write(0x00); // Minutes
    Wire1.write(0x00); // Hours (24-hour format)
    Wire1.write(0x06); // Day of the week (1 = Sunday, 6 = Saturday)
    Wire1.write(0x01); // Date
    Wire1.write(0x01); // Month
    Wire1.write(0x00); // Year (00 for 2000)

    Wire1.endTransmission();
    Serial.println("RTC reset to 00:00:00 01/01/2000 UTC.");
}

void readTimeFromRTC() {
    Wire1.beginTransmission(DS3231_ADDRESS);
    Wire1.write(0x00); // Set pointer to seconds register
    Wire1.endTransmission();

    Wire1.requestFrom(DS3231_ADDRESS, 7);
    if (Wire1.available() == 7) {
        uint8_t seconds = Wire1.read();
        uint8_t minutes = Wire1.read();
        uint8_t hours = Wire1.read();
        uint8_t dayOfWeek = Wire1.read();
        uint8_t date = Wire1.read();
        uint8_t month = Wire1.read();
        uint8_t year = Wire1.read();

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


void setTimeToRTC() {
    int sec, min, hour, dayOfWeek, date, month, year, century;

    // Prompt for seconds and write pointer + seconds immediately
    Serial.println("Enter seconds (0-59):");
    while (!Serial.available()) yield();
    sec = Serial.parseInt();
    Serial.println(sec);

    // Write pointer and seconds to RTC
    auto toBCD = [](int val) { return ((val / 10) << 4) | (val % 10); };
    Wire1.beginTransmission(DS3231_ADDRESS);
    Wire1.write(0x00); // Pointer to seconds register
    Wire1.write(toBCD(sec));
    Wire1.endTransmission();

    // Prompt for the rest of the time/date info
    Serial.println("Enter minutes (0-59):");
    while (!Serial.available()) yield();
    min = Serial.parseInt();
    Serial.println(min);

    Serial.println("Enter hours (0-23):");
    while (!Serial.available()) yield();
    hour = Serial.parseInt();
    Serial.println(hour);

    Serial.println("Enter day of week (1=Sun, 7=Sat):");
    while (!Serial.available()) yield();
    dayOfWeek = Serial.parseInt();
    Serial.println(dayOfWeek);

    Serial.println("Enter date (1-31):");
    while (!Serial.available()) yield();
    date = Serial.parseInt();
    Serial.println(date);

    Serial.println("Enter month (1-12):");
    while (!Serial.available()) yield();
    month = Serial.parseInt();
    Serial.println(month);

    Serial.println("Enter year (0-99):");
    while (!Serial.available()) yield();
    year = Serial.parseInt();
    Serial.println(year);

    Serial.println("Enter century (0 for 20xx, 1 for 21xx):");
    while (!Serial.available()) yield();
    century = Serial.parseInt();
    Serial.println(century);

    uint8_t monthByte = toBCD(month);
    if (century) monthByte |= 0x80; // Set century bit if needed

    // Write the rest of the time/date to RTC (starting at minutes register)
    Wire1.beginTransmission(DS3231_ADDRESS);
    Wire1.write(0x01); // Pointer to minutes register
    Wire1.write(toBCD(min));
    Wire1.write(toBCD(hour));
    Wire1.write(toBCD(dayOfWeek));
    Wire1.write(toBCD(date));
    Wire1.write(monthByte);
    Wire1.write(toBCD(year));
    Wire1.endTransmission();

    Serial.println("RTC set to user-specified time.");
}



void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    Wire1.setSDA(6);
    Wire1.setSCL(7);
    Wire1.setClock(400000); 

    Wire1.begin();
    Serial.println("RTC test started.");
    Serial.println("Enter 1 to reset to zero, 2 to read current time, or 3 to set time:");
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
            Serial.println("Reseting RTC to time 0...");
            resetTimeToRTC();
            break;
        case '2':
            Serial.println("Reading current time from RTC...");
            readTimeFromRTC();
            break;
        case '3':
            Serial.println("Set RTC to user-specified time...");
            setTimeToRTC();
            break;
        default:
            Serial.println("Invalid input. Enter 1, 2, or 3:");
            return;
    }

    // Prompt the user again
    Serial.println("Enter 1 to reset to zero, 2 to read current time, or 3 to set time:");
}