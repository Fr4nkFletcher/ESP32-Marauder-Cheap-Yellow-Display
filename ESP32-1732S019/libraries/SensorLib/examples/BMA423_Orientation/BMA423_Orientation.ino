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
 * @file      BMA423_Orientation.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-03
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBMA423.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  21
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  22
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  39
#endif

SensorBMA423 accel;
char report[256];

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(SENSOR_IRQ, INPUT);

    if (!accel.begin(Wire, BMA423_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find BMA423 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("Init BAM423 Sensor success!");

    //Default 4G ,200HZ
    accel.configAccelerometer();

    accel.enableAccelerometer();

    Serial.println("Output ...");
}



void loop()
{
    uint8_t direction = accel.direction();
    switch (direction) {
    case SensorBMA423::DIRECTION_BOTTOM_LEFT:

        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |  *             | " \
                 "\r\n |________________| \r\n" );

        break;
    case SensorBMA423::DIRECTION_TOP_RIGHT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |             *  | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |________________| \r\n" );
        break;
    case SensorBMA423::DIRECTION_TOP_LEFT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |  *             | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |________________| \r\n" );
        break;
    case SensorBMA423::DIRECTION_BOTTOM_RIGHT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |             *  | " \
                 "\r\n |________________| \r\n" );
        break;
    case SensorBMA423::DIRECTION_BOTTOM:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |________________| " \
                 "\r\n    *               \r\n" );
        break;
    case SensorBMA423::DIRECTION_TOP:
        sprintf( report, "\r\n  __*_____________  " \
                 "\r\n |________________| \r\n" );
        break;
    default:
        sprintf( report, "None of the 3D orientation axes is set in BMA423 - accelerometer.\r\n" );
        break;
    }

    Serial.println(direction);
    Serial.println(report);

    delay(1000);
}



