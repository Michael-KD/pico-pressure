#include <Arduino.h>

#define UART_3V3_EN_PIN 10
#define UART_5V_EN_PIN 11
#define GPS_EN_PIN 13

void clearSerialInput() {
    uint32_t m = micros();
    while (micros() - m < 10000) {
        if (Serial.read() >= 0) {
            m = micros();
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        yield();
    }
    delay(1000);

    // Set pin modes
    pinMode(UART_3V3_EN_PIN, OUTPUT);
    pinMode(UART_5V_EN_PIN, OUTPUT);
    pinMode(GPS_EN_PIN, OUTPUT);

    // Ensure all pins are LOW initially
    digitalWrite(UART_3V3_EN_PIN, LOW);
    digitalWrite(UART_5V_EN_PIN, LOW);
    digitalWrite(GPS_EN_PIN, LOW);

    Serial.println("Switch test started.");
    Serial.println("Enter 1 for 3V3 UART, 2 for 5V UART, or 3 for GPS:");
}

void loop() {
    clearSerialInput();

    Serial.println("\nWaiting for input...");
    while (!Serial.available()) {
        yield();
    }

    char input = Serial.read();

    // Turn all pins LOW before setting the selected one HIGH
    digitalWrite(UART_3V3_EN_PIN, LOW);
    digitalWrite(UART_5V_EN_PIN, LOW);
    digitalWrite(GPS_EN_PIN, LOW);

    switch (input) {
        case '1':
            Serial.println("3V3 UART selected. Press Enter to continue...");
            digitalWrite(UART_3V3_EN_PIN, HIGH);
            break;
        case '2':
            Serial.println("5V UART selected. Press Enter to continue...");
            digitalWrite(UART_5V_EN_PIN, HIGH);
            break;
        case '3':
            Serial.println("GPS selected. Press Enter to continue...");
            digitalWrite(GPS_EN_PIN, HIGH);
            break;
        default:
            Serial.println("Invalid input. Enter 1, 2, or 3:");
            return;
    }

    // Wait for any character
    while (!Serial.available()) {
        yield();
    }
    clearSerialInput(); // Clear any extra input after Enter is pressed

    // Prompt the user again
    Serial.println("Enter 1 for 3V3 UART, 2 for 5V UART, or 3 for GPS:");
}