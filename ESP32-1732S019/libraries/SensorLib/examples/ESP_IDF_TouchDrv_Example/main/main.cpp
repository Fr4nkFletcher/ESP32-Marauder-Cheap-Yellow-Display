/**
 * @file      main.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2024-01-10
 *
 */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "i2c_driver.h"

#define SENSOR_INPUT_PIN                   (gpio_num_t)CONFIG_SENSOR_IRQ    /*!< axp power chip interrupt Pin*/
#define SENSOR_INPUT_PIN_SEL               (1ULL<<SENSOR_INPUT_PIN)

static const char *TAG = "mian";

extern esp_err_t sensor_drv_init();
extern esp_err_t touch_drv_init();

extern void touch_drv_isr_handler();

static void sensor_hander_task(void *);
static QueueHandle_t  gpio_evt_queue = NULL;

static void IRAM_ATTR sensor_irq_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void irq_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = SENSOR_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_intr_type(SENSOR_INPUT_PIN, GPIO_INTR_NEGEDGE);
    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(SENSOR_INPUT_PIN, sensor_irq_handler, (void *) SENSOR_INPUT_PIN);

}

extern "C" void app_main(void)
{
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(5, sizeof(uint32_t));

    // Register Sensor interrupt pins
    irq_init();

    ESP_ERROR_CHECK(i2c_drv_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(touch_drv_init());

    xTaskCreate(sensor_hander_task, "App/sen", 4 * 1024, NULL, 10, NULL);

    ESP_LOGI(TAG, "Run...");

}


static void sensor_hander_task(void *args)
{
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            touch_drv_isr_handler();
        }
    }
}



