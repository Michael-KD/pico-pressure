// LORA radio
// UART0, TX is GPIO 1, RX is GPIO 2
// RYLR998 module

#include <Arduino.h>
#include <Wire.h>
#include "RYLR998.h"

LoRa lora(Serial1, 2000); // HW UART 0, pins 1 and 2


void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    Wire.begin();
    Serial.println("LORA test started.");
    lora.begin(115200);
    lora.setDebug(true);
    lora.test();
    delay(1000);    
    lora.send("AT+FACTORY");
    delay(1000);
    lora.send("AT+ADDRESS=1");
    delay(1000);

    Serial.println("Pressure sensor initialized.");
    delay(1000);

    lora.setReceiveCallback([](uint16_t address, const String& data, int rssi, int snr) {
        Serial.print("Received data from address ");
        Serial.print(address);
        Serial.print(": ");
        Serial.println(data);
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("SNR: ");
        Serial.println(snr);
    });
}

void loop() {
    lora.send("AT+SEND=0,5,Hello");
    delay(1000);
}