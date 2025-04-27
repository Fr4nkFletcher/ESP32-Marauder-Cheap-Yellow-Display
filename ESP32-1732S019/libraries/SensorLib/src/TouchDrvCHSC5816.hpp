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
 * @file      TouchDrvCHSC5816.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#pragma once

#include "REG/CHSC5816Constants.h"
#include "TouchDrvInterface.hpp"
#include "SensorCommon.tpp"

typedef struct __CHSC5816_Header {
    uint16_t fw_ver;
    uint16_t checksum;
    uint32_t sig;
    uint32_t vid_pid;
    uint16_t raw_offet;
    uint16_t dif_offet;
} CHSC5816_Header_t;

union __CHSC5816_PointReg {
    struct {
        uint8_t status;
        uint8_t fingerNumber;
        uint8_t x_l8;
        uint8_t y_l8;
        uint8_t z;
        uint8_t x_h4: 4;
        uint8_t y_h4: 4;
        uint8_t id: 4;
        uint8_t event: 4;
        uint8_t p2;
    } rp;
    unsigned char data[8];
};

class TouchDrvCHSC5816 :
    public TouchDrvInterface,
    public SensorCommon<TouchDrvCHSC5816>
{
    friend class SensorCommon<TouchDrvCHSC5816>;
public:


#if defined(ARDUINO)
    TouchDrvCHSC5816(PLATFORM_WIRE_TYPE &w,
                     int sda = DEFAULT_SDA,
                     int scl = DEFAULT_SCL,
                     uint8_t addr = CHSC5816_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = addr;
    }
#endif

    TouchDrvCHSC5816()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = CHSC5816_SLAVE_ADDRESS;
    }

    ~TouchDrvCHSC5816()
    {
        // deinit();
    }

#if defined(ARDUINO)
    bool begin(PLATFORM_WIRE_TYPE &w,
               uint8_t addr = CHSC5816_SLAVE_ADDRESS,
               int sda = DEFAULT_SDA,
               int scl = DEFAULT_SCL)
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }
#endif

#if defined(ESP_PLATFORM) && !defined(ARDUINO)
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr)
    {
        return SensorCommon::begin(i2c_dev_bus_handle, addr);
    }
#else
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
    {
        return SensorCommon::begin(port_num, addr, sda, scl);
    }
#endif //ESP_IDF_VERSION
#endif

    bool begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
    {
        return SensorCommon::begin(addr, readRegCallback, writeRegCallback);
    }

    void reset()
    {
        if (__rst != SENSOR_PIN_NONE) {
            this->setGpioLevel(__rst, LOW);
            delay(3);
            this->setGpioLevel(__rst, HIGH);
            delay(5);
        }
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)
    {
        __CHSC5816_PointReg touch;

        // CHSC5816_REG_POINT
        uint8_t write_buffer[] = {0x20, 0x00, 0x00, 0x2c};
        writeThenRead(write_buffer, SENSORLIB_COUNT(write_buffer), touch.data, 8);
        if (touch.rp.status == 0xFF && touch.rp.fingerNumber == 0) {
            return 0;
        }
        if (x_array) {
            *x_array = (unsigned int)(touch.rp.x_h4 << 8) | touch.rp.x_l8;
        }
        if (y_array) {
            *y_array = (unsigned int)(touch.rp.y_h4 << 8) | touch.rp.y_l8;
        }

        updateXY(1, x_array, y_array);

        return 1;
    }

    bool isPressed()
    {
        if (__irq != SENSOR_PIN_NONE) {
            return this->getGpioLevel(__irq) == LOW;
        }
        return getPoint(NULL, NULL);
    }

    const char *getModelName()
    {
        return "CHSC5816";
    }

    //2uA
    void sleep()
    {
        uint8_t write_buffer[] = {
            0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
            0xF8, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE9
        };
        writeBuffer(write_buffer, SENSORLIB_COUNT(write_buffer));
    }

    void wakeup()
    {
        reset();
    }

    void idle()
    {
        uint8_t write_buffer[] = {
            0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
            0x20, 0x16, 0x02, 0x00, 0xDB, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE9
        };
        writeBuffer(write_buffer, SENSORLIB_COUNT(write_buffer));
    }

    uint8_t getSupportTouchPoint()
    {
        return 1;
    }

    bool getResolution(int16_t *x, int16_t *y)
    {
        /*
        uint8_t buffer[16] = {
            0xFC, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xe9
        };
        writeRegister(CHSC5816_REG_CMD_BUFF, buffer, 16);
        memset(buffer, 0, 16);
        readRegister(CHSC5816_REG_CMD_BUFF, buffer, 16);
        for (int i = 0; i < 16; ++i) {
            Serial.print(buffer[i], HEX);
            Serial.print(",");
        }
        Serial.println();
        return true;
        */
        return false;
    }

    void  setGpioCallback(gpio_mode_fptr_t mode_cb,
                          gpio_write_fptr_t write_cb,
                          gpio_read_fptr_t read_cb)
    {
        SensorCommon::setGpioModeCallback(mode_cb);
        SensorCommon::setGpioWriteCallback(write_cb);
        SensorCommon::setGpioReadCallback(read_cb);
    }

private:
    bool checkOnline()
    {
        CHSC5816_Header_t tmp;
        memset(&tmp, 0, sizeof(CHSC5816_Header_t));
        memset(&__header, 0, sizeof(CHSC5816_Header_t));

        // CHSC5816_REG_BOOT_STATE 0x20000018
        uint8_t write_buffer[] = {0x20, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00};

        if (writeBuffer(write_buffer, SENSORLIB_COUNT(write_buffer)) == DEV_WIRE_ERR) {
            log_e("writeBuffer clean boot state failed!\n");
            return false;
        }

        reset();

        for (int i = 0; i < 10; ++i) {
            delay(10);
            // CHSC5816_REG_IMG_HEAD 0x20000014
            uint8_t write_buffer[] = {0x20, 0x00, 0x00, 0x14};
            if (writeThenRead(write_buffer, SENSORLIB_COUNT(write_buffer),
                              (uint8_t *)&__header,
                              sizeof(CHSC5816_Header_t)) == DEV_WIRE_ERR) {
                printf("readRegister 1 failed!\n");
                return false;
            }

            if (writeThenRead(write_buffer, SENSORLIB_COUNT(write_buffer),
                              (uint8_t *)&tmp,
                              sizeof(CHSC5816_Header_t)) == DEV_WIRE_ERR) {
                printf("readRegister 2 failed!\n");
                return false;
            }

            if (memcmp(&tmp, &__header, sizeof(CHSC5816_Header_t)) != 0 ) {
                continue;
            }
            if (__header.sig == CHSC5816_SIG_VALUE) {
                return true;
            }
        }
        return false;
    }

    bool initImpl()
    {

        if (__irq != SENSOR_PIN_NONE) {
            this->setGpioMode(__irq, INPUT);
        }

        if (__rst != SENSOR_PIN_NONE) {
            this->setGpioMode(__rst, OUTPUT);
        }

        reset();

        if (checkOnline()) {
            reset();
            return true;
        }

        return false;
    }

    int getReadMaskImpl()
    {
        return -1;
    }


protected:
    CHSC5816_Header_t __header;
};



