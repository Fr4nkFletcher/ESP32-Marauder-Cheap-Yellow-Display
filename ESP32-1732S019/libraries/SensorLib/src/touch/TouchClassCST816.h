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
 * @file      TouchClassCST816.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#pragma once

#include "../TouchDrvInterface.hpp"
#include "../SensorCommon.tpp"
#include "../SensorLib.h"

class TouchClassCST816 : public TouchDrvInterface,
    public SensorCommon<TouchClassCST816>
{
    friend class SensorCommon<TouchClassCST816>;

public:

#if defined(ARDUINO)
    TouchClassCST816();

    bool begin(PLATFORM_WIRE_TYPE &wire, uint8_t address, int sda, int scl);

#elif defined(ESP_PLATFORM)
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr);
#else
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl);
#endif //ESP_IDF_VERSION
#endif

    bool begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback);


    void reset();

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point);

    bool isPressed();

    const char *getModelName();

    void sleep();

    void wakeup();

    void idle();

    uint8_t getSupportTouchPoint();

    bool getResolution(int16_t *x, int16_t *y);

    void setCenterButtonCoordinate(int16_t x, int16_t y);

    void setHomeButtonCallback(home_button_callback_t cb, void *user_data);

    void disableAutoSleep();

    void enableAutoSleep();

    void setGpioCallback(gpio_mode_fptr_t mode_cb,
                         gpio_write_fptr_t write_cb,
                         gpio_read_fptr_t read_cb);

private:
    bool initImpl();
    int getReadMaskImpl();

protected:
    int16_t __center_btn_x;
    int16_t __center_btn_y;
};







