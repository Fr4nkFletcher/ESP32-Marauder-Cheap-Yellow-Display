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
 * @file      CM32181_LightSensor.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-14
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorCM32181.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  39
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  40
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  1
#endif

SensorCM32181 light;


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(SENSOR_IRQ, INPUT_PULLUP);

    if (!light.begin(Wire, CM32181_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find CM32181 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init CM32181 Sensor success!");

    int id =  light.getChipID();
    Serial.print("CM32181 ID = "); Serial.println(id);

    /*
        Sensitivity mode selection
            SAMPLING_X1
            SAMPLING_X2
            SAMPLING_X1_8
            SAMPLING_X1_4
    */
    light.setSampling(SensorCM32181::SAMPLING_X2);

    //Power On sensor
    light.powerOn();

}


void loop()
{
    // Get raw data
    uint16_t raw = light.getRaw();

    // Get conversion data , The manual does not provide information on how to obtain the
    //  calibration value, now use the calibration value 0.28 provided by the manual
    float lux = light.getLux();
    Serial.printf("RAW:%u Lux:%.2f\n", raw, lux);
    delay(500);
}



