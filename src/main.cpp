// #include <Arduino.h>
// #include <Wire.h>
// // #include <SD.h>
// #include <SPI.h>

#include <Arduino.h>
#include <RYLR998.h>

#define LORA_BAUD_RATE 115200
#define LORA_SERIAL Serial2 // Adjust this as needed for your setup

LoRa lora(LORA_SERIAL, 2000); // Create a LoRa object with a 2-second timeout

void onReceive(uint16_t address, const String& data, int rssi, int snr) {
    Serial.print("Received data from address ");
    Serial.print(address);
    Serial.print(": ");
    Serial.println(data);
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" SNR: ");
    Serial.println(snr);
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor
    while (!Serial); // Wait for serial monitor to open
    delay(1000); // Stabilize the serial monitor

    Serial.println("Starting LoRa module setup...");

    lora.setDebug(true); // Enable debug prints
    lora.begin(LORA_BAUD_RATE); // Initialize LoRa module serial
    delay(1000);

    lora.send("AT");
    lora.waitForResponse();
    
    lora.send("AT");
    lora.waitForResponse();
    

    // Set network ID and frequency
    lora.send("AT+FACTORY"); // Reset to factory settings
    lora.waitForResponse();
    lora.send("AT+NETWORKID=6");
    lora.waitForResponse();
    lora.send("AT+ADDRESS=1"); // Set address to 1
    lora.waitForResponse();
    lora.setReceiveCallback(onReceive); // Set the receive callback

    lora.queryVersion();

    Serial.println("LoRa module setup complete.");
}

void loop() {
    lora.waitForResponse(2000);
}