/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      PCF85063_ClockOutput.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-24
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorPCF85063.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  7
#endif


SensorPCF85063::ClockHz clock_array[] = {
    SensorPCF85063::CLK_32768HZ,
    SensorPCF85063::CLK_16384HZ,
    SensorPCF85063::CLK_8192HZ,
    SensorPCF85063::CLK_4096HZ,
    SensorPCF85063::CLK_2048HZ,
    SensorPCF85063::CLK_1024HZ,
    SensorPCF85063::CLK_1HZ,
    SensorPCF85063::CLK_LOW,
};

String freq_hz_str[] = {
    "32.768KHZ",
    "16.384KHZ",
    "8.192KHZ",
    "4.096KHZ",
    "2.048KHZ",
    "1.024KHZ",
    "1HZ",
    "LOW"
};


SensorPCF85063 rtc;
uint32_t lastMillis;
uint8_t i = 0;


void setup()
{
    Serial.begin(115200);
    while (!Serial);
    if (!rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF8563 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
}


void loop()
{
    if (millis() - lastMillis > 5000) {
        lastMillis = millis();
        Serial.print("Set freq : ");
        Serial.println(freq_hz_str[i]);
        rtc.setClockOutput(clock_array[i]);
        i++;
        i %= 8;
    }
}


