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
 * @file      BHI260AP_Orientation.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-07
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"

#ifdef BHY2_USE_I2C
#define BHI260AP_SDA          21
#define BHI260AP_SCL          22
#define BHI260AP_IRQ          39
#define BHI260AP_RST          -1
#else
#define BHI260AP_MOSI         33
#define BHI260AP_MISO         34
#define BHI260AP_SCK          35
#define BHI260AP_CS           36
#define BHI260AP_IRQ          37
#define BHI260AP_RST          47
#endif

void orientation_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp);

SensorBHI260AP bhy;

void setup()
{
    Serial.begin(115200);
    while (!Serial);


    // Set the reset pin and interrupt pin, if any
    bhy.setPins(BHI260AP_RST, BHI260AP_IRQ);

#ifdef BHY2_USE_I2C
    // Using I2C interface
    // BHI260AP_SLAVE_ADDRESS_L = 0x28
    // BHI260AP_SLAVE_ADDRESS_H = 0x29
    if (!bhy.init(Wire, BHI260AP_SDA, BHI260AP_SCL, BHI260AP_SLAVE_ADDRESS_L)) {
        Serial.print("Failed to init BHI260AP - ");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#else
    // Using SPI interface
    if (!bhy.init(SPI, BHI260AP_CS, BHI260AP_MOSI, BHI260AP_MISO, BHI260AP_SCK)) {
        Serial.print("Failed to init BHI260AP - ");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

    Serial.println("Init BHI260AP Sensor success!");

    // Output all available sensors to Serial
    bhy.printSensors(Serial);

    float sample_rate = 100.0;      /* Read out hintr_ctrl measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */

    // Direction detection relies on the accelerometer, and the accelerometer needs to be turned on first.
    bhy.configure(SENSOR_ID_ACC_PASS, sample_rate, report_latency_ms);
    // Enable direction detection
    bhy.configure(SENSOR_ID_DEVICE_ORI, sample_rate, report_latency_ms);
    // Set the direction detection result output processing function
    bhy.onResultEvent(SENSOR_ID_DEVICE_ORI, orientation_process_callback);
}


void loop()
{
    // Update sensor fifo
    bhy.update();
    delay(50);
}


void orientation_process_callback(uint8_t  sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    char report[256];
    uint8_t direction = *data_ptr;
    switch (direction) {
    case BHY2_DIRECTION_BOTTOM_LEFT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |  *             | " \
                 "\r\n |________________| \r\n" );

        break;
    case BHY2_DIRECTION_TOP_RIGHT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |             *  | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |________________| \r\n" );
        break;
    case BHY2_DIRECTION_TOP_LEFT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |  *             | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |________________| \r\n" );
        break;
    case BHY2_DIRECTION_BOTTOM_RIGHT:
        sprintf( report, "\r\n  ________________  " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |                | " \
                 "\r\n |             *  | " \
                 "\r\n |________________| \r\n" );
        break;
    default:
        sprintf( report, "None of the 3D orientation axes is set in BHI260 - accelerometer.\r\n" );
        break;
    }
    Serial.print(bhy.getSensorName(sensor_id));
    Serial.print(":");
    Serial.println(direction);
    Serial.println(report);
}
