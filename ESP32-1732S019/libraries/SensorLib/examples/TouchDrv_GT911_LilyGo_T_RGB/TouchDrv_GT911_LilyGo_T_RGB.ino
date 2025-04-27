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
 * @file      TouchDrv_GT911_LilyGo_T_RGB.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvGT911.hpp"
#include "ExtensionIOXL9555.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  8
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  48
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  1
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  1
#endif

TouchDrvGT911 touch;
ExtensionIOXL9555 extIO;
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


void digitalWrite_CB(uint32_t pin, uint8_t value)
{
    extIO.digitalWrite(pin, value);
}

void pinMode_CB(uint32_t pin, uint8_t mode)
{
    extIO.pinMode(pin, mode);
}



void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Start!");

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

    scanDevices();

    // T-RGB Use 0x20 device address
    if (!extIO.begin(Wire, XL9555_SLAVE_ADDRESS0, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find XL9555 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    // Set PORT0 as output
    extIO.configPort(ExtensionIOXL9555::PORT0, OUTPUT);
    extIO.writePort(ExtensionIOXL9555::PORT0, 0xFF);


    touch.setPins(SENSOR_RST, SENSOR_IRQ);
    touch.setGpioWriteCallback(digitalWrite_CB);
    touch.setGpioModeCallback(pinMode_CB);

    if (!touch.begin(Wire, GT911_SLAVE_ADDRESS_L, SENSOR_SDA, SENSOR_SCL )) {
        scanDevices();

        while (1) {
            Serial.println("Failed to find GT911 - check your wiring!");
            delay(1000);
        }
    }

    //Set to trigger on falling edge
    touch.setInterruptMode(FALLING);

    Serial.println("Init GT911 Sensor success!");

}

void loop()
{
    if (touch.isPressed()) {
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
    delay(5);
}




