/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      BMM150_GetDataExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-10
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBMM150.hpp"
#include "SensorWireHelper.h"

#ifndef SENSOR_SDA
#define SENSOR_SDA          33
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL          35
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ          -1
#endif

#ifndef SENSOR_RST
#define SENSOR_RST          -1
#endif


SensorBMM150 bmm;



void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Using I2C interface
    if (!bmm.init(Wire, SENSOR_SDA, SENSOR_SCL, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW)) {
        Serial.print("Failed to init BMM150 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init BMM150 Sensor success!");

    /* Set magnetometer run mode */
    /**
     * * POWERMODE_NORMAL,
     * * POWERMODE_FORCED,
     * * POWERMODE_SLEEP,
     * * POWERMODE_SUSPEND,
    */
    bmm.setMode(SensorBMM150::POWERMODE_NORMAL);

}


void loop()
{
    int16_t x, y, z;
    /* Read mag data */
    /* Unit for magnetometer data is microtesla(uT) */
    if (bmm.getMag(x, y, z)) {
        Serial.print("X:");
        Serial.print(x);
        Serial.print("uT,");
        Serial.print("Y:");
        Serial.print(y);
        Serial.print("uT,");
        Serial.print("Z:");
        Serial.print(z);
        Serial.println("uT");
    }
    delay(50);
}

