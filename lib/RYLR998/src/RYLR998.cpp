#include "RYLR998.h"

typedef void (*ReceiveCallback)(uint16_t address, String data, int rssi, int snr);

// Constructor
LoRa::LoRa(HardwareSerial &port, unsigned long responseTimeout)
    : serialPort(port), timeout(responseTimeout) {}

// Initialize the LoRa module
void LoRa::begin(unsigned long baudRate) {
    serialPort.begin(baudRate);
    if (debugEnabled) {
        Serial.println("LoRa module initialized.");
    }
}

// Send a command to the LoRa module
void LoRa::send(const String &command) {
    serialPort.print(command + "\r\n");
    if (debugEnabled) {
        Serial.print("Sent: ");
        Serial.println(command);
    }
}

// Wait for a response from the LoRa module
String LoRa::waitForResponse(unsigned long waitTime) {
    if (waitTime == 0) {
        waitTime = timeout;
    }
    unsigned long startTime = millis();
    while (millis() - startTime < waitTime) {
        if (serialPort.available()) {
            String response = serialPort.readString();
            if (debugEnabled) {
                Serial.print("Response: ");
                Serial.println(response);
            }
            if (response.startsWith("+RCV=")) {
                Serial.println("Received data from LoRa module.");
                // Parse the +RCV command
                int addressEnd = response.indexOf(',', 5);
                int lengthEnd = response.indexOf(',', addressEnd + 1);
                int dataEnd = response.indexOf(',', lengthEnd + 1);
                int rssiEnd = response.indexOf(',', dataEnd + 1);

                uint16_t address = response.substring(5, addressEnd).toInt();
                String data = response.substring(addressEnd + 1, lengthEnd);
                int rssi = response.substring(dataEnd + 1, rssiEnd).toInt();
                int snr = response.substring(rssiEnd + 1).toInt();

                if (receiveCallback) {
                    receiveCallback(address, data, rssi, snr);
                }
            }
            return response;
        }
    }

    if (debugEnabled) {
        Serial.println("No response.");
    }
    return "";
}

// Set the receive callback
void LoRa::setReceiveCallback(ReceiveCallback callback) {
    receiveCallback = callback;
}

void LoRa::queryVersion() {
    send("AT+VER?");
    waitForResponse();
}

void LoRa::reset() {
    send("AT+RESET");
    waitForResponse();
}

// Enable or disable debug prints
void LoRa::setDebug(bool enable) {
    debugEnabled = enable;
}