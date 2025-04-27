/**
 * @file      i2c_driver.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2024-01-10
 *
 */
#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API)
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif  //ESP_IDF_VERSION


esp_err_t i2c_drv_init(void);
void i2c_drv_scan();
bool  i2c_drv_probe(uint8_t devAddr);
int i2c_read_callback(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);
int i2c_write_callback(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);

#if CONFIG_I2C_COMMUNICATION_METHOD_CALLBACK_RW
esp_err_t i2c_drv_device_init(uint8_t address);
#endif


