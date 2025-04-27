/**
 * @file      esp_arduino.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2024-01-08
 *
 */
#if !defined(ARDUINO)  && defined(ESP_PLATFORM)
#include "SensorLib.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <rom/ets_sys.h>


void pinMode(uint32_t gpio, uint8_t mode)
{
    gpio_config_t config;
    memset(&config, 0, sizeof(config));
    config.pin_bit_mask = 1ULL << gpio;
    switch (mode) {
    case INPUT:
        config.mode = GPIO_MODE_INPUT;
        break;
    case OUTPUT:
        config.mode = GPIO_MODE_OUTPUT;
        break;
    }
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
}

void digitalWrite(uint32_t gpio, uint8_t level)
{
    gpio_set_level((gpio_num_t )gpio, level);
}

int digitalRead(uint32_t gpio)
{
    return gpio_get_level((gpio_num_t)gpio);
}

void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
    ets_delay_us((ms % portTICK_PERIOD_MS) * 1000UL);
}

uint32_t millis()
{
    return (uint32_t) (esp_timer_get_time() / 1000LL);
}

uint32_t micros()
{
    return (uint32_t) esp_timer_get_time();
}

void delayMicroseconds(uint32_t us)
{
    ets_delay_us(us);
}
#endif
