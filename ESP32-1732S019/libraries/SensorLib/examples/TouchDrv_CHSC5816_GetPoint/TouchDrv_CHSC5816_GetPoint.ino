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
 * @file      TouchDrv_CHSC5816_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-17
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvCHSC5816.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  39
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  40
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  13
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  14
#endif

TouchDrvCHSC5816 touch;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    touch.setPins(SENSOR_RST, SENSOR_IRQ);
    if (!touch.begin(Wire, CHSC5816_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find CHSC5816 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init CHSC5816 Touch device success!");
}


void loop()
{
    int16_t x[2], y[2];
    if (digitalRead(SENSOR_IRQ) == LOW) {
        uint8_t touched = touch.getPoint(x, y);
        for (int i = 0; i < touched; ++i) {
            Serial.print("X[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(x[i]);
            Serial.print(" ");
            Serial.print(" Y[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(y[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
}



