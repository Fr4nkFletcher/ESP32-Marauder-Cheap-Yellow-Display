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
 * @file      BHI260AP_aux_BMM150_euler.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-23
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"

/*
Write the firmware containing the BMM150 magnetometer function into the flash.
This function requires the BHI260AP external SPI Flash.
If there is no Flash, it can only be written and run in RAM.
Example firmware source: https://github.com/boschsensortec/BHY2_SensorAPI/tree/master/firmware
You can also compile custom firmware to write
How to build custom firmware see : https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bhi260ab-an000.pdf
*/
#define WRITE_TO_FLASH          1           //Set 1 write fw to flash ,set 0 write fw to ram

#if   WRITE_TO_FLASH
#include "BHI260AP_aux_BMM150-flash.fw.h"
const uint8_t *firmware = bhi26ap_aux_bmm150_flash_fw;
const size_t fw_size = sizeof(bhi26ap_aux_bmm150_flash_fw);

#else
#include "BHI260AP_aux_BMM150.fw.h"
const uint8_t *firmware = bhi26ap_aux_bmm150_fw;
const size_t fw_size = sizeof(bhi26ap_aux_bmm150_fw);
#endif

#ifdef BHY2_USE_I2C
#define BHI260AP_SDA          21
#define BHI260AP_SCL          22
#define BHI260AP_IRQ          39
#define BHI260AP_RST          -1
#else
#define BHI260AP_MOSI         27
#define BHI260AP_MISO         46
#define BHI260AP_SCK          3
#define BHI260AP_CS           28
#define BHI260AP_IRQ          30
#define BHI260AP_RST          -1
#endif


SensorBHI260AP bhy;


void parse_euler(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    struct bhy2_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;

    // Function to parse FIFO frame data into orientation
    bhy2_parse_orientation(data_ptr, &data);

    uint64_t _timestamp =  *timestamp;
    time_to_s_ns(_timestamp, &s, &ns, &tns);

    uint8_t accuracy =  bhy.getAccuracy();
    if (accuracy) {

        Serial.print("SID:"); Serial.print(sensor_id);
        Serial.print(" T:"); Serial.print(s);
        Serial.print("."); Serial.print(ns);
        Serial.print(" x:"); Serial.print(data.heading * 360.0f / 32768.0f);
        Serial.print(" y:"); Serial.print(data.pitch * 360.0f / 32768.0f);
        Serial.print(" x:"); Serial.print(data.roll * 360.0f / 32768.0f);
        Serial.print(" acc:"); Serial.print(accuracy);
        /*
        Serial.printf("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f; acc: %u\r\n",
                      sensor_id,
                      s,
                      ns,
                      data.heading * 360.0f / 32768.0f,
                      data.pitch * 360.0f / 32768.0f,
                      data.roll * 360.0f / 32768.0f,
                      accuracy);
        */
    } else {

        Serial.print("SID:"); Serial.print(sensor_id);
        Serial.print(" T:"); Serial.print(s);
        Serial.print("."); Serial.print(ns);
        Serial.print(" x:"); Serial.print(data.heading * 360.0f / 32768.0f);
        Serial.print(" y:"); Serial.print(data.pitch * 360.0f / 32768.0f);
        Serial.print(" x:"); Serial.print(data.roll * 360.0f / 32768.0f);
        /*
        Serial.printf("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f\r\n",
                      sensor_id,
                      s,
                      ns,
                      data.heading * 360.0f / 32768.0f,
                      data.pitch * 360.0f / 32768.0f,
                      data.roll * 360.0f / 32768.0f);
        */
    }
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin and interrupt pin, if any
    bhy.setPins(BHI260AP_RST, BHI260AP_IRQ);

    // Set the firmware array address and firmware size
    bhy.setFirmware(firmware, fw_size, WRITE_TO_FLASH);

#if WRITE_TO_FLASH
    // Set to load firmware from flash
    bhy.setBootFormFlash(true);
#endif

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

    /*
    * Enable Euler function
    * The Euler function depends on BMM150.
    * If the hardware is not connected to BMM150, the Euler function cannot be used.
    * * */
    bhy.configure(SENSOR_EULER_ID, sample_rate, report_latency_ms);

    // Register event callback function
    bhy.onResultEvent(SENSOR_EULER_ID, parse_euler);


}


void loop()
{
    // Update sensor fifo
    bhy.update();
    delay(50);
}



