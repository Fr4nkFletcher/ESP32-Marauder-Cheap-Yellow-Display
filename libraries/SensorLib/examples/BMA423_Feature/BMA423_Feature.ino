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
 * @file      BMA423_Feature.ino
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
uint32_t lastMillis;
bool sensorIRQ = false;


void setFlag()
{
    sensorIRQ = true;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(SENSOR_IRQ, INPUT);
    attachInterrupt(SENSOR_IRQ, setFlag, RISING);

    if (!accel.begin(Wire, BMA423_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find BMA423 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init BAM423 Sensor success!");

    //Default 4G ,200HZ
    accel.configAccelerometer();

    // Enable acceleration sensor
    accel.enableAccelerometer();

    // Enable pedometer steps
    accel.enablePedometer();

    // Emptying the pedometer steps
    accel.resetPedometer();

    // Enable sensor features
    accel.enableFeature(SensorBMA423::FEATURE_STEP_CNTR |
                        SensorBMA423::FEATURE_ANY_MOTION |
                        SensorBMA423::FEATURE_ACTIVITY |
                        SensorBMA423::FEATURE_TILT |
                        SensorBMA423::FEATURE_WAKEUP,
                        true);

    // Pedometer interrupt enable
    accel.enablePedometerIRQ();
    // Tilt interrupt enable
    accel.enableTiltIRQ();
    // DoubleTap interrupt enable
    accel.enableWakeupIRQ();
    // Any  motion / no motion interrupt enable
    accel.enableAnyNoMotionIRQ();
    // Activity interruption enable
    accel.enableActivityIRQ();
    // Chip interrupt function enable
    accel.configInterrupt();

}


void loop()
{
    if (sensorIRQ) {
        sensorIRQ = false;
        // The interrupt status must be read after an interrupt is detected
        uint16_t status =   accel.readIrqStatus();
        Serial.printf("Accelerometer interrupt mask : 0x%x\n", status);

        if (accel.isPedometer()) {
            uint32_t stepCounter = accel.getPedometerCounter();
            Serial.print("Step count interrupt,step Counter:");
            Serial.println(stepCounter);
        }
        if (accel.isActivity()) {
            Serial.println("Activity interrupt");
        }
        if (accel.isTilt()) {
            Serial.println("Tilt interrupt");
        }
        if (accel.isDoubleTap()) {
            Serial.println("DoubleTap interrupt");
        }
        if (accel.isAnyNoMotion()) {
            Serial.println("Any motion / no motion interrupt");
        }
    }
    delay(50);
}



