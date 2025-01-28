#ifndef LORA_H
#define LORA_H

#include <Arduino.h>

class LoRa {
public:
    // Constructor
    LoRa(HardwareSerial &port, unsigned long responseTimeout);

    void begin(unsigned long baudRate);
    void send(const String &command);
    String waitForResponse(unsigned long waitTime = 0);

    void queryVersion();
    void reset();

    void setDebug(bool enable);

    // Callback type for received data
    typedef void (*ReceiveCallback)(uint16_t address, const String& data, int rssi, int snr);

    // Set the receive callback
    void setReceiveCallback(ReceiveCallback callback);

private:
    HardwareSerial &serialPort;    // Reference to the serial port
    unsigned long timeout;         // Timeout for waiting for responses
    bool debugEnabled = false;     // Flag to enable/disable debug prints
    ReceiveCallback receiveCallback = nullptr; // Callback for received data
};

#endif // LORA_H