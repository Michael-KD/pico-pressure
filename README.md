### Pico pressure V2.0 Testing
- PlatformIO
    - platform: maxgerhardt/platform-raspberrypi.git
    - board_build.core: earlephilhower (framework-arduinopico)

- Pin defs
    - c:\Users\micha\.platformio\packages\framework-arduinopico\variants\rpipico\pins_arduino.h

- need to set Wire1 sda, scl, and speed
    - Wire1.setSDA(6);
    - Wire1.setSCL(7);
    - Wire1.setClock(400000);
    - Wire1.begin();


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

- EEPROM test
    - eeprom_test.cpp
    - 24LC32A
    - I2C address 0x50
    - tested reading and writing 1 byte and power down

- RTC test
    - rtc_test.cpp
    - DS3231
    - I2C address 0x68
    - tested setting and reading time and date
    - Want to check alarm and temperature

#### To Do
- Pressure sensor test
    - I2C
        - I2C address 0x76
        - 500Hz with 256 OSR, clock speed 400kHz, 600us delay
    - SPI
        - 
    - use current library?
    - try new one, use 3 separate files vs 1


- UART header test
    - lora_test.cpp
    - turning on and off the lora module
    - 2 talking to each other
    - using one as a relay to increase the range

- GPS test
    - 


- Watchdog test
    - HB pulse pin 12

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