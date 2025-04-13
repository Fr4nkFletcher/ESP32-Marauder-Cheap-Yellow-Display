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
 * @file      PCF8563_AlarmByUnits.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-11
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <time.h>
#include "SensorPCF8563.hpp"

#ifdef ESP32
#define SENSOR_SDA                     42
#define SENSOR_SCL                     41
#define SENSOR_IRQ                     14
#else
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#define SENSOR_SDA             _PINNUM(0,26)
#define SENSOR_SCL             _PINNUM(0,27)
#define SENSOR_IRQ             _PINNUM(0,16)
#endif


SensorPCF8563 rtc;


uint32_t lastMillis = 0;
uint8_t nextHour = 22;
uint8_t nextMonth = 1;
uint8_t nextDay = 1;
uint8_t nextMinute = 59;
uint8_t nextSecond = 55;


void setup()
{
    Serial.begin(115200);
    while (!Serial);



    if (!rtc.begin(Wire, PCF8563_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF8563 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    pinMode(SENSOR_IRQ, INPUT_PULLUP);

    rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);

    // From minute timer
    rtc.setAlarmByMinutes(0);

    rtc.enableAlarm();

}

void printDateTime()
{
    if (millis() - lastMillis > 1000) {
        /**
        /// Format output time*
        Option:
            DATETIME_FORMAT_HM
            DATETIME_FORMAT_HMS
            DATETIME_FORMAT_YYYY_MM_DD
            DATETIME_FORMAT_MM_DD_YYYY
            DATETIME_FORMAT_DD_MM_YYYY
            DATETIME_FORMAT_YYYY_MM_DD_H_M_S
        default:   DATETIME_FORMAT_YYYY_MM_DD_H_M_S_WEEK
        */
        Serial.println(rtc.strftime());

        lastMillis = millis();
    }
}

// Test minute timing
void  testAlarmMinute()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmMinute Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                nextHour++;
                if (nextHour >= 24) {
                    nextHour = 23;
                    nextDay = 25;
                    rtc.setAlarmByHours(0);
                    Serial.println("setAlarmByHours");
                    return;
                }
            }
        }
        printDateTime();
    }
}

// Test hour timing
void  testAlarmHour()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmHour Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                nextDay++;
                if (nextDay >= 30) {
                    nextMonth = 1;
                    nextHour = 23;
                    nextMinute = 59;
                    nextSecond = 55;
                    nextDay = rtc.getDaysInMonth(nextMonth, 2022);
                    rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                    rtc.setAlarmByDays(1);
                    Serial.println("setAlarmByDays");
                    return;
                }
            }
        }
        printDateTime();
    }
}

// Test day timing
void  testAlarmDay()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmDay Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                nextDay = rtc.getDaysInMonth(nextMonth, 2022);
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                nextMonth++;
                if (nextMonth >= 12) {
                    return;
                }
            }
        }
        printDateTime();
    }
}




void loop()
{
    testAlarmMinute();
    testAlarmHour();
    testAlarmDay();

    Serial.println("Test done ...");
    while (1) {
        delay(100);
    }
}



