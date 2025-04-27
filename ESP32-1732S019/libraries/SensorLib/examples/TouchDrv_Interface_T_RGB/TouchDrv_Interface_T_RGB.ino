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
 * @file      TouchDrv_LilyGo_T_RGB.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-01-21
 * @note      T-RGB has three types of screens, each of which uses different touch driver chips. 
 *            The example demonstrates using the touch interface class and one sketch is suitable for three types of touch chips.
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "ExtensionIOXL9555.hpp"
#include "TouchDrvGT911.hpp"
#include "TouchDrvFT6X36.hpp"
#include "TouchDrvCSTXXX.hpp"



#ifndef SENSOR_SDA
#define SENSOR_SDA  8
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  48
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  1   //ESP32S3 GPIO1
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  1   //XL9555 GPIO1
#endif

TouchDrvInterface *touchDrv = NULL;
ExtensionIOXL9555 extension;

ExtensionIOXL9555::ExtensionGPIO tp_reset = ExtensionIOXL9555::IO1;
ExtensionIOXL9555::ExtensionGPIO power_enable = ExtensionIOXL9555::IO2;

int16_t x[5], y[5];


//GPIO callback function
void TouchDrvDigitalWrite(uint32_t gpio, uint8_t level)
{
    if (gpio & 0x80) {
        extension.digitalWrite(gpio & 0x7F, level);
    } else {
        digitalWrite(gpio, level);
    }
}

//GPIO callback function
int TouchDrvDigitalRead(uint32_t gpio)
{
    if (gpio & 0x80) {
        return extension.digitalRead(gpio & 0x7F);
    } else {
        return digitalRead(gpio);
    }
}

//GPIO callback function
void TouchDrvPinMode(uint32_t gpio, uint8_t mode)
{
    if (gpio & 0x80) {
        extension.pinMode(gpio & 0x7F, mode);
    } else {
        pinMode(gpio, mode);
    }
}


bool setupTouchDrv()
{
    //Touch interrupt uses ESP32S3 GPIO1, touch reset uses XL9555 extended IO chip GPIO1, and needs to add a mask to distinguish
    const uint8_t touch_reset_pin = 1 | 0x80;
    const uint8_t touch_irq_pin = 1;
    bool result = false;

    touchDrv = new TouchDrvCSTXXX();
    // T-RGB reset GPIO is connected through the expansion chip, and the GPIO callback function needs to be set.
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(Wire, CST816_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
    if (result) {
        const char *model = touchDrv->getModelName();
        Serial.printf("Successfully initialized %s, using %s Driver!", model, model);
        return true;
    }

    delete touchDrv;

    touchDrv = new TouchDrvGT911();
    // T-RGB reset GPIO is connected through the expansion chip, and the GPIO callback function needs to be set.
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(Wire, GT911_SLAVE_ADDRESS_H, SENSOR_SDA, SENSOR_SCL);
    if (result) {
        Serial.println("Successfully initialized GT911, using GT911 Driver!");
        return true;
    }
    delete touchDrv;

    touchDrv = new TouchDrvFT6X36();
    // T-RGB reset GPIO is connected through the expansion chip, and the GPIO callback function needs to be set.
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(Wire, FT3267_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
    if (result) {
        const char *model = touchDrv->getModelName();
        Serial.printf("Successfully initialized %s, using %s Driver!", model, model);
        return true;
    }
    delete touchDrv;

    return false;
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

    // T-RGB XL9555 Use 0x20 device address
    if (!extension.begin(Wire, XL9555_SLAVE_ADDRESS0, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find XL9555 - check your wiring!");
            delay(1000);
        }
    }

    // T-RGB power enable GPIO connected to XL9555 expansion chip GPIO
    extension.pinMode(power_enable, OUTPUT);
    extension.digitalWrite(power_enable, HIGH);

    // Initialize touch using touch driver interface class
    if (!setupTouchDrv()) {
        while (1) {
            Serial.println("Failed to find touch - check your wiring!");
            delay(1000);
        }
    }
}

void loop()
{
    if (touchDrv) {
        if (touchDrv->isPressed()) {
        uint8_t touched = touchDrv->getPoint(x, y, touchDrv->getSupportTouchPoint());
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
    }
    delay(5);
}




