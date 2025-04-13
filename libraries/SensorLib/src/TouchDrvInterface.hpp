/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted,free of charge,to any person obtaining a copy
 * of this software and associated documentation files (the "Software"),to deal
 * in the Software without restriction,including without limitation the rights
 * to use,copy,modify,merge,publish,distribute,sublicense,and/or sell
 * copies of the Software,and to permit persons to whom the Software is
 * furnished to do so,subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS",WITHOUT WARRANTY OF ANY KIND,EXPRESS OR
 * IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,DAMAGES OR OTHER
 * LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE,ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      TouchDrvInterface.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-17
 *
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#include "SensorLib.h"
#include "SensorCommon.tpp"

typedef void    (*home_button_callback_t)(void *user_data);


class TouchData
{
public:
    TouchData() {}
    ~TouchData() {};
    uint8_t available;
    uint8_t id[5];
    int16_t x[5];
    int16_t y[5];
    uint8_t  status[5];
    uint8_t  pressure[5];

    uint16_t getX(uint8_t index = 0)
    {
        return x[index];
    }
    uint16_t getY(uint8_t index = 0)
    {
        return y[index];
    }
    uint16_t getPressure(uint8_t index = 0)
    {
        return pressure[index];
    }
    uint16_t getStatus(uint8_t index = 0)
    {
        return status[index];
    }
};



class TouchDrvInterface
{
public:
    TouchDrvInterface();

    virtual ~TouchDrvInterface();

#if defined(ARDUINO)

    virtual bool begin(PLATFORM_WIRE_TYPE &wire, uint8_t address, int sda, int scl) = 0;

#elif defined(ESP_PLATFORM) && !defined(ARDUINO)

#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    virtual bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr) = 0;
#else
    virtual bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl) = 0;
#endif //ESP_IDF_VERSION

#endif //defined(ESP_PLATFORM) && !defined(ARDUINO)

    virtual bool begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)  = 0;

    virtual void reset() = 0;

    virtual uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point) = 0;

    virtual bool isPressed() = 0;

    virtual const char *getModelName() = 0;

    virtual void sleep() = 0;

    virtual void wakeup() = 0;

    virtual void idle() = 0;

    virtual uint8_t getSupportTouchPoint() = 0;

    virtual bool getResolution(int16_t *x, int16_t *y) = 0;

    virtual void setGpioCallback(gpio_mode_fptr_t mode_cb,
                                 gpio_write_fptr_t write_cb,
                                 gpio_read_fptr_t read_cb) = 0;

    uint32_t getChipID();

    void setPins(int rst, int irq);

    void setSwapXY(bool swap);

    void setMirrorXY(bool mirrorX, bool mirrorY);

    void setMaxCoordinates(uint16_t x, uint16_t y);

    void updateXY(uint8_t pointNum, int16_t *xBuffer, int16_t *yBuffer);

protected:
    uint16_t __resX, __resY, __xMax, __yMax;
    bool __swapXY, __mirrorX, __mirrorY;
    int __rst;
    int __irq;
    uint32_t __chipID;
    home_button_callback_t __homeButtonCb;
    void *__userData;

};







