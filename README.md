Pico pressure V2.0 Testing
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


- UART header test
    - lora_test.cpp
    - turning on and off the lora module
    - 2 talking to each other
    - using one as a relay to increase the range

- GPS test
    - 

- RTC test
    - 

- Watchdog test
    - HB pulse pin 12

- Pressure sensor test
    - try I2C and SPI

- Dual core test
    - core 0 and core 1
    - try using spi on both cores, different pins/different devices?