/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      PCF8563_SimpleTime.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-12
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorPCF8563.hpp"

// lilygo t-beam-s3-core pin
#define SENSOR_SDA                     42
#define SENSOR_SCL                     41
#define SENSOR_IRQ                     14

SensorPCF8563 rtc;
uint32_t lastMillis;

void setup()
{
    Serial.begin(115200);
    while (!Serial);



    pinMode(SENSOR_IRQ, INPUT_PULLUP);

    if (!rtc.begin(Wire, PCF8563_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF8563 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    uint16_t year = 2022;
    uint8_t month = 12;
    uint8_t day = 12;
    uint8_t hour = 21;
    uint8_t minute = 00;
    uint8_t second = 30;

    rtc.setDateTime(year, month, day, hour, minute, second);

}


void loop()
{
    if (millis() - lastMillis > 1000) {
        lastMillis = millis();
        RTC_DateTime datetime = rtc.getDateTime();
        Serial.printf(" Year :"); Serial.print(datetime.year);
        Serial.printf(" Month:"); Serial.print(datetime.month);
        Serial.printf(" Day :"); Serial.print(datetime.day);
        Serial.printf(" Hour:"); Serial.print(datetime.hour);
        Serial.printf(" Minute:"); Serial.print(datetime.minute);
        Serial.printf(" Sec :"); Serial.println(datetime.second);

    }
}



