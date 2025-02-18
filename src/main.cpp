#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include "pico/multicore.h"

RTC_DS3231 rtc;


const char* daysOfTheWeek[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(100);
  Serial.println("RTC test");


  Wire.begin();
  rtc.begin();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // The following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // Alternatively, you can set the RTC with an explicit date & time
    // rtc.adjust(DateTime(2023, 10, 5, 12, 0, 0));
  }

}

void loop () {
    DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    
    delay(1000);
}



void setup1() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop1() {
  Serial.println("Hello from core 1!");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}