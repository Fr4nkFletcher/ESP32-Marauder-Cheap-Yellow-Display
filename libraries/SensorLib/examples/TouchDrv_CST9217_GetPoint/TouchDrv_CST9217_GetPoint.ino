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
 * @file      TouchDrv_CST9217_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-27
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvCST92xx.h"
#include "SensorWireHelper.h"


#ifndef SENSOR_SDA
#define SENSOR_SDA  8
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  10
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  5
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  -1
#endif

TouchDrvCST92xx touch;
int16_t x[5], y[5];
bool  isPressed = false;


void setup()
{
    Serial.begin(115200);
    while (!Serial);

#if SENSOR_RST != -1
    pinMode(SENSOR_RST, OUTPUT);
    digitalWrite(SENSOR_RST, LOW);
    delay(30);
    digitalWrite(SENSOR_RST, HIGH);
    delay(50);
    delay(1000);
#endif

#if defined(ARDUINO_ARCH_RP2040)
    Wire.setSCL(SENSOR_SCL);
    Wire.setSDA(SENSOR_SDA);
    Wire.begin();
#elif defined(NRF52840_XXAA) || defined(NRF52832_XXAA)
    Wire.setPins(SENSOR_SDA, SENSOR_SCL);
    Wire.begin();
#else
    Wire.begin(SENSOR_SDA, SENSOR_SCL);
#endif

    // Scan I2C devices
    SensorWireHelper::dumpDevices(Wire);


    // uint8_t touchAddress = 0x1A;        // Other device addresses are determined by the touch firmware and are generally 0X5A by default.
    uint8_t touchAddress = 0x5A;     // The device address is determined according to the actual situation. Not all device addresses are 0X5A. There can also be other customized device addresses.
    

    // Set to skip register check, used when the touch device address conflicts with other I2C device addresses [0x5A]
    touch.jumpCheck();
    

    touch.setPins(SENSOR_RST, SENSOR_IRQ);
    bool result = touch.begin(Wire, touchAddress, SENSOR_SDA, SENSOR_SCL);
    if (result == false) {
        Serial.println("touch is not online..."); while (1)delay(1000);
    }
    Serial.print("Model :"); Serial.println(touch.getModelName());

    touch.setCoverScreenCallback([](void *ptr) {
        Serial.print(millis());
        Serial.println(" : The screen is covered");
    }, NULL);


    Serial.println("Enter touch sleep mode.");
    // Unable to obtain coordinates after turning on sleep
    // CST9217 Work current ~= 1.3mA
    // CST9217 sleep current = 3.4 uA
    touch.sleep();

    int i = 10;
    while (i--) {
        Serial.printf("Wake up after %d seconds\n", i);
        delay(1000);
    }

    // Wakeup touch
    touch.reset();

    // Set touch max xy
    // touch.setMaxCoordinates(240, 296);

    // Set swap xy
    // touch.setSwapXY(true);

    // Set mirror xy
    // touch.setMirrorXY(true, true);

    //Register touch plane interrupt pin
    attachInterrupt(SENSOR_IRQ, []() {
        isPressed = true;
    }, FALLING);
}

void loop()
{
    if (isPressed) {
        isPressed = false;
        uint8_t touched = touch.getPoint(x, y, touch.getSupportTouchPoint());
        if (touched) {
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
    delay(30);
}



