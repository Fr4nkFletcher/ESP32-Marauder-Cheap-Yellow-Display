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
 * @file      BHI260AP_aux_BMM150_BME280.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-24
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

#if WRITE_TO_FLASH
#include "BHI260_aux_BMM150_BME280-flash.fw.h"
const uint8_t *firmware = bhi26ap_aux_bmm150_bme280_flash_fw;
const size_t fw_size = sizeof(bhi26ap_aux_bmm150_bme280_flash_fw);

#else
#include "BHI260_aux_BMM150_BME280.fw.h"
const uint8_t *firmware = bhi26ap_aux_bmm150_bme280_fw;
const size_t fw_size = sizeof(bhi26ap_aux_bmm150_bme280_fw);
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


void parse_bme280_sensor_data(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    float humidity = 0;
    float temperature = 0;
    float pressure = 0;
    switch (sensor_id) {
    case SENSOR_ID_HUM:
        bhy2_parse_humidity(data_ptr, &humidity);
        Serial.print("humidity:"); Serial.print(humidity); Serial.println("%");
        break;
    case SENSOR_ID_TEMP:
        bhy2_parse_temperature_celsius(data_ptr, &temperature);
        Serial.print("temperature:"); Serial.print(temperature); Serial.println("*C");
        break;
    case SENSOR_ID_BARO:
        bhy2_parse_pressure(data_ptr, &pressure);
        Serial.print("pressure:"); Serial.print(pressure); Serial.println("hPa");
        break;
    default:
        break;
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

    float sample_rate = 0.0;      /* Read out hintr_ctrl measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */

    /*
    * Enable BME280 function
    * Function depends on BME280.
    * If the hardware is not connected to BME280, the function cannot be used.
    * * */
    bhy.configure(SENSOR_ID_TEMP, sample_rate, report_latency_ms);
    bhy.configure(SENSOR_ID_HUM, sample_rate, report_latency_ms);
    bhy.configure(SENSOR_ID_BARO, sample_rate, report_latency_ms);


    // Register BME280 data parse callback function
    bhy.onResultEvent(SENSOR_ID_TEMP, parse_bme280_sensor_data);
    bhy.onResultEvent(SENSOR_ID_HUM, parse_bme280_sensor_data);
    bhy.onResultEvent(SENSOR_ID_BARO, parse_bme280_sensor_data);
}


void loop()
{
    // Update sensor fifo
    bhy.update();
    delay(50);
}



