### Pico pressure V2.0 Testing
- PlatformIO
    - platform: maxgerhardt/platform-raspberrypi.git
    - board_build.core: earlephilhower (framework-arduinopico)

#### Done

- SD Card test
    - SD_test.cpp
    - Sdfat - Adafruit fork
    - CS pin 17
    - Format to exFAT
    - works with SD_SCK_MHZ(50)
    - still need to test reading and writing files and speeds


- Switch tests
    - switch_test.cpp
    - 3V3 uart
        - EN pin 10
        - works
    - 5V uart
        - EN pin 11
        - works
    - GPS
        - EN pin 13
        - works

#### To Do

- RTC test
    - rtc_test.cpp
    - DS3231
    - I2C address 0x68
    - **file not tested
    - Want to test
        - set time and date
        - read time and date
        - set alarm
        - read alarm
        - set temperature?
        - power down and wake up

- EEPROM test
    - eeprom_test.cpp
    - 24LC32A
    - I2C address 0x50
    - **file not tested
    - Want to test
        - write and read data
        - write and read multiple bytes
        - write and read different data types (int, float, char, etc.)
        - write and read different sizes of data (1 byte, 2 bytes, 4 bytes, etc.)
        - power down and wake up

- UART header test
    - lora_test.cpp
    - turning on and off the lora module
    - 2 talking to each other
    - using one as a relay to increase the range

- GPS test
    - 


- Watchdog test
    - HB pulse pin 12

- Pressure sensor test
    - try I2C and SPI
    - use current library?
    - try new one, use 3 separate files vs 1

- Dual core test
    - core 0 and core 1
    - try using spi on both cores, different pins/different devices?


- Write (or find) libraries for everything
    - SPI
        - SD card
            - [Sdfat - Adafruit fork](https://github.com/adafruit/SdFat)
    - I2C
        - RTC (DS3231)
            - [RTCLib - Adafruit fork](https://github.com/adafruit/RTClib)
            - needs adafruit BusIO?
        - EEPROM (24LC32A)
            - [AdaFruit_EEPROM](https://github.com/adafruit/Adafruit_FRAM_I2C)
        - GPS (MAX-M10S)
    - UART
        - Lora (RYLR998)
        
    - Watchdog 

    - Pressure sensor