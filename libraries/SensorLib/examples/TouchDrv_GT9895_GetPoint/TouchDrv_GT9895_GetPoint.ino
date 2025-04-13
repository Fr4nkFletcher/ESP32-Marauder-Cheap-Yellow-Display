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
 * @file      TouchDrv_GT9895_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#include <Arduino.h>
#include "TouchDrvGT9895.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  2
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  3
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  1
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  10
#endif

TouchDrvGT9895 touch;

int16_t x[GT9895_MAX_TOUCH], y[GT9895_MAX_TOUCH];

void setup()
{
    Serial.begin(115200);

    while (!Serial);

    delay(3000);

    // Set touch interrupt and reset Pin
    touch.setPins(SENSOR_RST, SENSOR_IRQ);

    if (!touch.begin(Wire, GT9895_SLAVE_ADDRESS_L, SENSOR_SDA, SENSOR_SCL )) {
        while (1) {
            Serial.println("Failed to find GT9895 - check your wiring!");
            delay(1000);
        }
    }

    Serial.print("Chip ID : 0x"); Serial.println(touch.getChipID(), HEX);

    // Sleep touch
    // touch.sleep();

    // int i = 10;
    // while (i--) {
    //     Serial.printf("Wake up after %d seconds\n", i);
    //     delay(1000);
    // }

    // Wakeup touch
    // touch.wakeup();

    // Set touch max xy
    // touch.setMaxCoordinates(240, 296);

    // Set swap xy
    // touch.setSwapXY(true);

    // Set mirror xy
    // touch.setMirrorXY(true, true);

}

void loop()
{
    /*
    * Methods to increase the refresh rate:
    * 1. Set the touch interrupt to a low level when touching, rather than a falling edge.
    *    Some are falling edge triggered, with a trigger period of about 10ms, depending on the firmware
    *    This requires the touch screen manufacturer to provide a firmware update, which is currently unavailable.
    * 2. Use polling registers instead of interrupts, but this will consume CPU
    */
    if (touch.isPressed()) {        //IRQ Trigger

        uint8_t touched = touch.getPoint(x, y, touch.getSupportTouchPoint());
        if (touched > 0) {
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

