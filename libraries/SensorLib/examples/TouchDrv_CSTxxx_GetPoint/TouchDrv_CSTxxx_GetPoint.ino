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
 * @file      TouchDrv_CSTxxx_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvCSTXXX.hpp"

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

TouchDrvCSTXXX touch;
int16_t x[5], y[5];

void scanDevices(void)
{
    byte error, address;
    int nDevices = 0;
    Serial.println("Scanning for I2C devices ...");
    for (address = 0x01; address < 0x7f; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X\n", address);
            nDevices++;
        } else if (error != 2) {
            Serial.printf("Error %d at address 0x%02X\n", error, address);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    }
}

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
    // delay(1000);
#endif

    // Search for known CSTxxx device addresses
    uint8_t address = 0xFF;

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
    scanDevices();

    Wire.beginTransmission(CST816_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        address = CST816_SLAVE_ADDRESS;
    }
    Wire.beginTransmission(CST226SE_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        address = CST226SE_SLAVE_ADDRESS;
    }
    Wire.beginTransmission(CST328_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        address = CST328_SLAVE_ADDRESS;
    }
    while (address == 0xFF) {
        Serial.println("Could't find touch chip!"); delay(1000);
    }

    touch.setPins(SENSOR_RST, SENSOR_IRQ);
    touch.begin(Wire, address, SENSOR_SDA, SENSOR_SCL);


    Serial.print("Model :"); Serial.println(touch.getModelName());

    // T-Display-S3 CST816 touch panel, touch button coordinates are is 85 , 160
    touch.setCenterButtonCoordinate(85, 360);

    // T-Display-AMOLED 1.91 Inch CST816T touch panel, touch button coordinates is 600, 120.
    // touch.setCenterButtonCoordinate(600, 120);  // Only suitable for AMOLED 1.91 inch


    // Depending on the touch panel, not all touch panels have touch buttons.
    touch.setHomeButtonCallback([](void *user_data) {
        Serial.println("Home key pressed!");
    }, NULL);


    // Unable to obtain coordinates after turning on sleep
    // CST816T sleep current = 1.1 uA
    // CST226SE sleep current = 60 uA
    // touch.sleep();

    // Set touch max xy
    // touch.setMaxCoordinates(536, 240);

    // Set swap xy
    // touch.setSwapXY(true);

    // Set mirror xy
    // touch.setMirrorXY(true, true);

}

void loop()
{
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

    delay(5);
}



