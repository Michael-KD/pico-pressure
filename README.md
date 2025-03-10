# Setup steps for now:
- format sd card to fat32
    - still need to test file sizes/limit
    - try exFat library?
    - Downloads\guiformat-x64\guiformat-x64.exe

- to get pico uploading working
    - zadig
    - WinUSB drivers for the "RP2 Boot2 (Interface 1)" device

- check final current used by the pico
- clock speed of pico, check frequency read
- try binary log files


1091mAh for ~26 hours
2:54 to 4:41
1:47 hours
58mAh in 1:47 hours = 34.3mA


# Mk II
- gps
    - GT-U7
    - MAX-M10S
- rtc
    - DS3231M
- lora (maybe)
    - RYLR998, think theyre broken? order more?
    - serial
- rp2040 vs rp2350?
- dual core
    - one for pressure sensor
    - one for everything else
    - look into DMA
- 2 i2c buses
    - one for pressure sensor
    - one for rtc, gps, etc?
