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
 * @file      TouchDrvCSTXXX.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 * @date      last 2023-10-05
 *
 */
#pragma once

#include "REG/CSTxxxConstants.h"
#include "touch/TouchClassCST226.h"
#include "touch/TouchClassCST816.h"
#include "SensorCommon.tpp"

class TouchDrvCSTXXX : public TouchDrvInterface
{
public:
    TouchDrvCSTXXX(): drv(NULL)
    {
    }

    ~TouchDrvCSTXXX()
    {
        if (drv) {
            delete drv;
            drv = NULL;
        }
    }

#if defined(ARDUINO)
    bool begin(PLATFORM_WIRE_TYPE &wire,
               uint8_t address,
               int sda,
               int scl
              )
    {
        if (!drv) {
            drv = new TouchClassCST816();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(wire, address, sda, scl)) {
                delete drv;
                drv = NULL;
            }
        }

        if (!drv) {
            drv = new TouchClassCST226();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(wire, address, sda, scl)) {
                delete drv;
                drv = NULL;
            }
        }

        return drv != NULL;
    }
#elif defined(ESP_PLATFORM)
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr)
    {
        if (!drv) {
            drv = new TouchClassCST816();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(i2c_dev_bus_handle, addr)) {
                delete drv;
                drv = NULL;
            }
        }

        if (!drv) {
            drv = new TouchClassCST226();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(i2c_dev_bus_handle, addr)) {
                delete drv;
                drv = NULL;
            }
        }
        return drv != NULL;
    }
#else
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
    {
        if (!drv) {
            drv = new TouchClassCST816();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(port_num, addr, sda, scl)) {
                delete drv;
                drv = NULL;
            }
        }

        if (!drv) {
            drv = new TouchClassCST226();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(port_num, addr, sda, scl)) {
                delete drv;
                drv = NULL;
            }
        }
        return drv != NULL;
    }
#endif //ESP_IDF_VERSION
#endif//ARDUINO


    void setGpioCallback(gpio_mode_fptr_t mode_cb,
                         gpio_write_fptr_t write_cb,
                         gpio_read_fptr_t read_cb)
    {
        __set_gpio_level = write_cb;
        __get_gpio_level = read_cb;
        __set_gpio_mode = mode_cb;
    }

    bool begin(uint8_t address, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
    {
        if (!drv) {
            drv = new TouchClassCST816();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(address, readRegCallback, writeRegCallback)) {
                delete drv;
                drv = NULL;
            }
        }

        if (!drv) {
            drv = new TouchClassCST226();
            drv->setGpioCallback(__set_gpio_mode, __set_gpio_level, __get_gpio_level);
            drv->setPins(__rst, __irq);
            if (!drv->begin(address, readRegCallback, writeRegCallback)) {
                delete drv;
                drv = NULL;
            }
        }

        return drv != NULL;
    }


    void reset()
    {
        if (!drv)return;
        drv->reset();
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)
    {
        if (!drv)return 0;
        return drv->getPoint(x_array, y_array, get_point);
    }

    bool isPressed()
    {
        if (!drv)return false;
        return drv->isPressed();
    }

    const char *getModelName()
    {
        if (!drv)return "NULL";
        return drv->getModelName();
    }

    void sleep()
    {
        if (!drv)return;
        drv->sleep();
    }

    void wakeup()
    {
        if (!drv)return;
        drv->reset();
    }

    void idle()
    {
        if (!drv)return;
        drv->idle();
    }

    uint8_t getSupportTouchPoint()
    {
        if (!drv)return 0;
        return drv->getSupportTouchPoint();
    }

    bool getResolution(int16_t *x, int16_t *y)
    {
        if (!drv)return false;
        return drv->getResolution(x, y);
    }

    void setCenterButtonCoordinate(uint16_t x, uint16_t y)
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->setCenterButtonCoordinate(x, y);
        }
    }

    void setHomeButtonCallback(home_button_callback_t callback, void *user_data = NULL)
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->setHomeButtonCallback(callback, user_data);
            // pT->setCenterButtonCoordinate(600, 120);  // Only suitable for AMOLED 1.91 inch

        } if (strncmp(model, "CST2", 3) == 0) {
            TouchClassCST226 *pT = static_cast<TouchClassCST226 *>(drv);
            pT->setHomeButtonCallback(callback, user_data);
        }
    }

    void disableAutoSleep()
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->disableAutoSleep();
        }
    }

    void enableAutoSleep()
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->enableAutoSleep();
        }
    }

    void setSwapXY(bool swap)
    {
        if (!drv)return ;
        drv->setSwapXY(swap);
    }

    void setMirrorXY(bool mirrorX, bool mirrorY)
    {
        if (!drv)return ;
        drv->setMirrorXY(mirrorX, mirrorY);
    }

    void setMaxCoordinates(uint16_t x, uint16_t y)
    {
        if (!drv)return ;
        drv->setMaxCoordinates(x, y);
    }

private:
    gpio_write_fptr_t   __set_gpio_level        = NULL;
    gpio_read_fptr_t    __get_gpio_level        = NULL;
    gpio_mode_fptr_t    __set_gpio_mode         = NULL;
    TouchDrvInterface *drv = NULL;
};



