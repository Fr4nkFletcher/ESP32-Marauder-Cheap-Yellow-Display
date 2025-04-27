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
 * @file      BHI260AP_aux_BMM150.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-22
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


void bhy_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    struct bhy2_data_xyz data;
    float scaling_factor = get_sensor_default_scaling(sensor_id);
    bhy2_parse_xyz(data_ptr, &data);
    Serial.print(bhy.getSensorName(sensor_id));
    Serial.print(":");
    Serial.printf("x: %f, y: %f, z: %f;\r\n",
                  data.x * scaling_factor,
                  data.y * scaling_factor,
                  data.z * scaling_factor
                 );
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
    uint32_t report_latency_ms = 0; /* 0 = report immediately */

    // Enable acceleration
    report_latency_ms = 1000;   // Report once per second
    bhy.configure(SENSOR_ID_ACC_PASS, sample_rate, report_latency_ms);

    // Enable gyroscope
    report_latency_ms = 1000;   //Report once per second
    bhy.configure(SENSOR_ID_GYRO_PASS, sample_rate, report_latency_ms);

    // Enable magnetometer
    report_latency_ms = 500;    //Report every 500 milliseconds
    bhy.configure(SENSOR_ID_MAG_PASS, sample_rate, report_latency_ms);

    // Set the acceleration sensor result callback function
    bhy.onResultEvent(SENSOR_ID_ACC_PASS, bhy_process_callback);

    // Set the gyroscope sensor result callback function
    bhy.onResultEvent(SENSOR_ID_GYRO_PASS, bhy_process_callback);

    // Set the magnetometer sensor result callback function
    bhy.onResultEvent(SENSOR_ID_MAG_PASS, bhy_process_callback);

}


void loop()
{
    // Update sensor fifo
    bhy.update();
    delay(50);
}



