/**
 * @file      esp_arduino.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2024-01-08
 *
 */
#pragma once

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)

void pinMode(uint32_t gpio, uint8_t mode);
void digitalWrite(uint32_t gpio, uint8_t level);
int digitalRead(uint32_t gpio);
void delay(uint32_t ms);
uint32_t millis();
uint32_t micros();
void delayMicroseconds(uint32_t us);

#endif

