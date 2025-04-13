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
 * @file      SensorLib.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-05
 */

#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#elif defined(ESP_PLATFORM)
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_err.h"
#include <cstring>
#include "esp_idf_version.h"
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif  //ESP_IDF_VERSION
#else
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#endif

#ifdef ARDUINO_ARCH_MBED
// Not supported at the moment
#error The Arduino RP2040 MBED board package is not supported when PIO is used. Use the community package by Earle Philhower.
#endif

#if defined(ARDUINO)
#if defined(ARDUINO_ARCH_RP2040)
#define PLATFORM_SPI_TYPE               SPIClassRP2040
#define PLATFORM_WIRE_TYPE              TwoWire
#define SPI_DATA_ORDER                  SPI_MSB_FIRST
#define DEFAULT_SDA                     (0xFF)
#define DEFAULT_SCL                     (0xFF)
#define DEFAULT_SPISETTING              SPISettings()
#elif defined(ARDUINO_ARCH_STM32)
#define PLATFORM_SPI_TYPE               SPIClass
#define PLATFORM_WIRE_TYPE              TwoWire
#define SPI_DATA_ORDER                  MSBFIRST
#define DEFAULT_SDA                     (0xFF)
#define DEFAULT_SCL                     (0xFF)
#define DEFAULT_SPISETTING              SPISettings()
#elif defined(ARDUINO_ARCH_NRF52)
#define PLATFORM_SPI_TYPE               SPIClass
#define PLATFORM_WIRE_TYPE              TwoWire
#define SPI_DATA_ORDER                  MSBFIRST
#define DEFAULT_SDA                     (0xFF)
#define DEFAULT_SCL                     (0xFF)
#define DEFAULT_SPISETTING              SPISettings()
#else
#define PLATFORM_SPI_TYPE               SPIClass
#define PLATFORM_WIRE_TYPE              TwoWire
#define SPI_DATA_ORDER                  SPI_MSBFIRST
#define DEFAULT_SDA                     (SDA)
#define DEFAULT_SCL                     (SCL)
#define DEFAULT_SPISETTING              SPISettings(__freq, __dataOrder, __dataMode);
#endif

#elif defined(ESP_PLATFORM)

#define SENSORLIB_I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define SENSORLIB_I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define SENSORLIB_I2C_MASTER_TIMEOUT_MS       1000
#define SENSORLIB_I2C_MASTER_SEEED            400000

#endif

#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH               (32)
#endif

enum SensorLibInterface {
    SENSORLIB_SPI_INTERFACE = 1,
    SENSORLIB_I2C_INTERFACE
};

typedef struct __SensorLibPins {
    int irq;
    int rst;
    union   __ {
        struct  {
            int sda;
            int scl;
            int addr;
#ifdef ARDUINO
            PLATFORM_WIRE_TYPE *wire;
#endif
        } i2c_dev;
        struct  {
            int cs;
            int miso;
            int mosi;
            int sck;
#ifdef ARDUINO
            PLATFORM_SPI_TYPE *spi;
#endif
        } spi_dev;
    } u  ;
    SensorLibInterface intf;
} SensorLibConfigure;




#define SENSOR_PIN_NONE     (-1)
#define DEV_WIRE_NONE       (0)
#define DEV_WIRE_ERR        (-1)
#define DEV_WIRE_TIMEOUT    (-2)




#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#ifdef _BV
#undef _BV
#endif
#define _BV(b)                          (1UL << (uint32_t)(b))

// #define LOG_PORT Serial
#ifdef LOG_PORT
#define LOG(fmt, ...) LOG_PORT.printf("[%s] " fmt "\n", __func__, ##__VA_ARGS__)
#define LOG_BIN(x)    LOG_PORT.println(x,BIN);
#else
#define LOG(fmt, ...) printf("[%s] " fmt "\n", __func__, ##__VA_ARGS__)
#define LOG_BIN(x)    printf("[%s] 0x%X\n", __func__, x)
#endif

#ifndef lowByte
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#endif

#ifndef highByte
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif

#ifndef bitSet
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#endif

#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif

#ifndef bitToggle
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#endif

#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

#ifndef isBitSet
#define isBitSet(value, bit)    (((value) & (1UL << (bit))) == (1UL << (bit)))
#endif

#define SENSORLIB_ATTR_NOT_IMPLEMENTED    __attribute__((error("Not implemented")))

#define SENSORLIB_COUNT(x)      (sizeof(x)/sizeof(*x))

#ifdef ARDUINO
#if !defined(ESP32) || !defined(ARDUINO_ARCH_ESP32)
#define LOG_FILE_LINE_INFO __FILE__, __LINE__
#ifndef log_e
#define log_e(fmt, ...)     Serial.printf("[E][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif
#ifndef log_i
#define log_i(fmt, ...)     Serial.printf("[I][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif
#ifndef log_d
#define log_d(fmt, ...)     Serial.printf("[D][%s:%d] " fmt "\n", LOG_FILE_LINE_INFO, ##__VA_ARGS__)
#endif
#endif
#elif defined(ESP_PLATFORM)
#define log_e(...)          printf(__VA_ARGS__)
#define log_i(...)          printf(__VA_ARGS__)
#define log_d(...)          printf(__VA_ARGS__)
#else
#define log_e(...)
#define log_i(...)
#define log_d(...)
#endif

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)

#ifndef INPUT
#define INPUT                 (0x0)
#endif

#ifndef OUTPUT
#define OUTPUT                (0x1)
#endif

#ifndef RISING
#define RISING                (0x01)
#endif

#ifndef FALLING
#define FALLING               (0x02)
#endif

#ifndef LOW
#define LOW 0
#endif

#ifndef HIGH
#define HIGH 1
#endif

#include "platform/esp_arduino.h"

#endif


#include "DevicesPins.h"