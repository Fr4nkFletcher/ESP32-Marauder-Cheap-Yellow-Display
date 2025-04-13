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
 * @file      TouchDrvGT911.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#pragma once

#include "REG/GT911Constants.h"
#include "TouchDrvInterface.hpp"
#include "SensorCommon.tpp"


#if defined(ARDUINO_ARCH_NRF52)
// NRF52840 I2C BUFFER : 64 Bytes ,
#warning "NRF Platform I2C Buffer expansion is not implemented , GT911 requires at least 188 bytes to read all configurations"
#endif

typedef struct GT911_Struct {
    uint8_t trackID;
    int16_t x;
    int16_t y;
    int16_t size;
} GT911Point_t;


#define LOW_LEVEL_QUERY         0x03
#define HIGH_LEVEL_QUERY        0x04

class TouchDrvGT911 :
    public TouchDrvInterface,
    public SensorCommon<TouchDrvGT911>
{
    friend class SensorCommon<TouchDrvGT911>;
public:


#if defined(ARDUINO)
    TouchDrvGT911(PLATFORM_WIRE_TYPE &w,
                  int sda = DEFAULT_SDA,
                  int scl = DEFAULT_SCL,
                  uint8_t addr = GT911_SLAVE_ADDRESS_H)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = addr;
    }
#endif

    TouchDrvGT911()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = GT911_SLAVE_ADDRESS_H;
    }

    ~TouchDrvGT911()
    {
        deinit();
    }

#if defined(ARDUINO)
    bool begin(PLATFORM_WIRE_TYPE &w,
               uint8_t addr = GT911_SLAVE_ADDRESS_H,
               int sda = DEFAULT_SDA,
               int scl = DEFAULT_SCL
              )
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }

#elif defined(ESP_PLATFORM) && !defined(ARDUINO)

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


    void deinit()
    {
        // end();
    }


    void reset()
    {
        if (__rst != SENSOR_PIN_NONE) {
            this->setGpioMode(__rst, OUTPUT);
            this->setGpioLevel(__rst, HIGH);
            delay(10);
        }
        if (__irq != SENSOR_PIN_NONE) {
            this->setGpioMode(__irq, INPUT);
        }
        /*
        * If you perform a software reset on a board without a reset pin connected,
        * subsequent interrupt settings or re-writing of configurations will be invalid.
        * For example, when debugging a LilyGo T-Deck, resetting the interrupt mode will
        * be invalid after a software reset.
        * */
        // writeRegister(GT911_COMMAND, 0x02);
        // writeCommand(0x02);
    }

    void sleep()
    {
        if (__irq != SENSOR_PIN_NONE) {
            this->setGpioMode(__irq, OUTPUT);
            this->setGpioLevel(__irq, LOW);
        }
        // writeRegister(GT911_COMMAND, 0x05);
        writeCommand(0x05);

        /*
        * Depending on the chip and platform, setting it to input after removing sleep will affect power consumption.
        * The chip platform determines whether
        *
        * * */
        // if (__irq != SENSOR_PIN_NONE) {
        //     this->setGpioLevel(__irq, INPUT);
        // }
    }



    void wakeup()
    {
        if (__irq != SENSOR_PIN_NONE) {
            this->setGpioMode(__irq, OUTPUT);
            this->setGpioLevel(__irq, HIGH);
            delay(8);
            this->setGpioMode(__irq, INPUT);
        } else {
            reset();
        }
    }

    void idle()
    {

    }

    uint8_t getSupportTouchPoint()
    {
        return 5;
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1)
    {
        uint8_t buffer[39];
        uint8_t touchPoint = 0;
        GT911Point_t p[5];

        if (!x_array || !y_array || size == 0)
            return 0;

        uint8_t val = readGT911(GT911_POINT_INFO);

        bool haveKey = GT911_GET_HAVE_KEY(val);
        // bool bufferStatus = GT911_GET_BUFFER_STATUS(val);
        // log_i("REG:0x%X S:0X%d K:%d\n", val,bufferStatus,haveKey);

        if (__homeButtonCb && haveKey) {
            __homeButtonCb(__userData);
        }

        clearBuffer();

        touchPoint = GT911_GET_POINT(val);
        if (touchPoint == 0) {
            return 0;
        }

        // GT911_POINT_1  0X814F
        uint8_t write_buffer[2] = {0x81, 0x4F};
        if (writeThenRead(write_buffer, SENSORLIB_COUNT(write_buffer),
                          buffer, 39) == DEV_WIRE_ERR) {
            return 0;
        }

        for (uint8_t i = 0; i < size; i++) {
            p[i].trackID = buffer[i * 8];
            p[i].x =  buffer[0x01 + i * 8] ;
            p[i].x |= (buffer[0x02 + i * 8] << 8 );
            p[i].y =  buffer[0x03 + i * 8] ;
            p[i].y |= (buffer[0x04 + i * 8] << 8);
            p[i].size = buffer[0x05 + i * 8] ;
            p[i].size |= (buffer[0x06 + i * 8] << 8) ;

            x_array[i] = p[i].x;
            y_array[i] = p[i].y;
        }

#ifdef LOG_PORT
        LOG_PORT.println("---------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        LOG_PORT.println("Touched  [0]ID  [0]Size  [0]X   [0]Y  [1]ID  [1]Size   [1]X    [1]Y  [2]ID  [2]Size   [2]X    [2]Y  [3]ID  [3]Size  [3]X    [3]Y  [4]ID  [4]Size  [4]X    [4]Y  ");
        LOG_PORT.print(touchPoint); LOG_PORT.print("\t");
        for (int i = 0; i < size; ++i) {
            LOG_PORT.print(p[i].trackID); LOG_PORT.print("\t");
            LOG_PORT.print(p[i].size); LOG_PORT.print("\t");
            LOG_PORT.print( p[i].x); LOG_PORT.print("\t");
            LOG_PORT.print( p[i].y); LOG_PORT.print("\t");
        }
        LOG_PORT.println();
#endif

        updateXY(touchPoint, x_array, y_array);

        return touchPoint;
    }


    bool isPressed()
    {
        if (__irq != SENSOR_PIN_NONE) {
            if (__irq_mode == FALLING) {
                return this->getGpioLevel(__irq) == LOW;
            } else if (__irq_mode == RISING ) {
                return this->getGpioLevel(__irq) == HIGH;
            } else if (__irq_mode == LOW_LEVEL_QUERY) {
                return this->getGpioLevel(__irq) == LOW;
            }  else if (__irq_mode == HIGH_LEVEL_QUERY) {
                return this->getGpioLevel(__irq) == HIGH;
            }
        }
        return getPoint();
    }

    bool setInterruptMode(uint8_t mode)
    {
        // GT911_MODULE_SWITCH_1 0x804D
        uint8_t val = readGT911(GT911_MODULE_SWITCH_1);
        val &= 0XFC;
        if (mode == FALLING) {
            val |= 0x01;
        } else if (mode == RISING ) {
            val |= 0x00;
        } else if (mode == LOW_LEVEL_QUERY ) {
            val |= 0x02;
        } else if (mode == HIGH_LEVEL_QUERY ) {
            val |= 0x03;
        }
        __irq_mode = mode;
        writeGT911(GT911_MODULE_SWITCH_1, val);
        return reloadConfig();
    }

    /**
     * @retval
     *  * 0x0: Rising edge trigger
     *  * 0x1: Falling edge trigger
     *  * 0x2: Low level query
     *  * 0x3: High level query
     */
    uint8_t getInterruptMode()
    {
        uint8_t val = readGT911(GT911_MODULE_SWITCH_1);
        // return val & 0x03;
        val &= 0x03;
        if (val == 0x00) {
            __irq_mode = RISING;
        } else if (val == 0x01) {
            __irq_mode = FALLING;
        } else if (val == 0x02) {
            __irq_mode = LOW_LEVEL_QUERY;
        } else if (val == 0x03) {
            __irq_mode = HIGH_LEVEL_QUERY;
        }
        return val;
    }


    uint8_t getPoint()
    {
        // GT911_POINT_INFO 0X814E
        uint8_t val = readGT911(GT911_POINT_INFO);
        clearBuffer();
        return GT911_GET_POINT(val);
    }


    uint32_t getChipID()
    {
        char product_id[4] = {0};
        // GT911_PRODUCT_ID 0x8140
        for (int i = 0; i < 4; ++i) {
            product_id[i] = readGT911(GT911_PRODUCT_ID + i);
        }
        return atoi(product_id);
    }

    uint16_t getFwVersion()
    {
        uint8_t fw_ver[2] = {0};
        // GT911_FIRMWARE_VERSION 0x8144
        for (int i = 0; i < 2; ++i) {
            fw_ver[i] = readGT911(GT911_FIRMWARE_VERSION + i);
        }
        return fw_ver[0] | (fw_ver[1] << 8);
    }

    uint8_t getConfigVersion()
    {
        return readGT911(GT911_CONFIG_VERSION);
    }


    bool getResolution(int16_t *x, int16_t *y)
    {
        uint8_t x_resolution[2] = {0}, y_resolution[2] = {0};

        for (int i = 0; i < 2; ++i) {
            x_resolution[i] = readGT911(GT911_X_RESOLUTION + i);
        }
        for (int i = 0; i < 2; ++i) {
            y_resolution[i] = readGT911(GT911_Y_RESOLUTION + i);
        }

        *x = x_resolution[0] | (x_resolution[1] << 8);
        *y = y_resolution[0] | (y_resolution[1] << 8);
        return true;
    }

    //Range : 5 ~ 15 ms
    void updateRefreshRate(uint8_t rate_ms)
    {
        if ((rate_ms - 5) < 5) {
            rate_ms = 5;
        }
        if (rate_ms > 15) {
            rate_ms = 15;
        }
        rate_ms -= 5;
        writeGT911(GT911_REFRESH_RATE, rate_ms);
        reloadConfig();
    }

    uint8_t getRefreshRate()
    {
        uint8_t rate_ms  = readGT911(GT911_REFRESH_RATE);
        return rate_ms + GT911_BASE_REF_RATE ;
    }


    int getVendorID()
    {
        return readGT911(GT911_VENDOR_ID);
    }


    const char *getModelName()
    {
        return "GT911";
    }

    void  setGpioCallback(gpio_mode_fptr_t mode_cb,
                          gpio_write_fptr_t write_cb,
                          gpio_read_fptr_t read_cb)
    {
        SensorCommon::setGpioModeCallback(mode_cb);
        SensorCommon::setGpioWriteCallback(write_cb);
        SensorCommon::setGpioReadCallback(read_cb);
    }

    void setHomeButtonCallback(home_button_callback_t cb, void *user_data)
    {
        __homeButtonCb = cb;
        __userData = user_data;
    }

    bool writeConfig(const uint8_t *config_buffer, size_t buffer_size)
    {
        setRegAddressLength(2);
        int err =  writeRegister(GT911_CONFIG_VERSION, (uint8_t *)config_buffer, buffer_size);
        setRegAddressLength(1);
        return err == DEV_WIRE_NONE;
    }

    uint8_t *loadConfig(size_t *output_size, bool print_out = false)
    {
        *output_size = 0;
        uint8_t   *buffer = (uint8_t * )malloc(GT911_REG_LENGTH * sizeof(uint8_t));
        if (!buffer)return NULL;
        uint8_t write_buffer[2] = {highByte(GT911_CONFIG_VERSION), lowByte(GT911_CONFIG_VERSION)};
        if (writeThenRead(write_buffer, SENSORLIB_COUNT(write_buffer), buffer, GT911_REG_LENGTH) == DEV_WIRE_ERR) {
            free(buffer);
            return NULL;
        }
        if (print_out) {
            printf("const unsigned char config[186] = {");
            for (int i = 0; i < GT911_REG_LENGTH; ++i) {
                if ( (i % 8) == 0) {
                    printf("\n");
                }
                printf("0x%02X", buffer[i]);
                if ((i + 1) < GT911_REG_LENGTH) {
                    printf(",");
                }
            }
            printf("};\n");
        }
        *output_size = GT911_REG_LENGTH;
        return buffer;
    }

    bool reloadConfig()
    {
        uint8_t buffer[GT911_REG_LENGTH] = {highByte(GT911_CONFIG_VERSION), lowByte(GT911_CONFIG_VERSION)};
        if (writeThenRead(buffer, 2, buffer, GT911_REG_LENGTH - 2) == DEV_WIRE_ERR) {
            return false;
        }

        uint8_t check_sum = 0;
        for (int i = 0; i < (GT911_REG_LENGTH - 2 ); i++) {
            check_sum += buffer[i];
        }
        check_sum =  (~check_sum) + 1;
        log_d("reloadConfig check_sum : 0x%X\n", check_sum);
        writeGT911(GT911_CONFIG_CHKSUM, check_sum);
        writeGT911(GT911_CONFIG_FRESH, 0x01);
        return true;
    }

    void dumpRegister()
    {
        size_t output_size = 0;
        uint8_t *buffer = loadConfig(&output_size, true);
        if (output_size == 0) {
            return;
        }

        if (buffer == NULL)return;
        printf("----------Dump register------------\n");
        for (size_t  i = 0; i < output_size; ++i) {
            printf("[%d]  REG: 0x%X : 0x%02X\n", i, GT911_CONFIG_VERSION + i, buffer[i]);
        }
        free(buffer);
    }

    // Range : 1~5
    void setMaxTouchPoint(uint8_t num)
    {
        if (num < 1)num = 1;
        if (num > 5) num = 5;
        writeGT911(GT911_TOUCH_NUMBER, num);
        reloadConfig();
    }

    uint8_t getMaxTouchPoint()
    {
        uint8_t num = readGT911(GT911_TOUCH_NUMBER);
        return num & 0x0F;
    }

private:

    uint8_t readGT911(uint16_t cmd)
    {
        uint8_t value = 0x00;
        uint8_t write_buffer[2] = {highByte(cmd), lowByte(cmd)};
        writeThenRead(write_buffer, SENSORLIB_COUNT(write_buffer),
                      &value, 1);
        return value;
    }

    int writeGT911(uint16_t cmd, uint8_t value)
    {
        uint8_t write_buffer[3] = {highByte(cmd), lowByte(cmd), value};
        return writeBuffer(write_buffer, SENSORLIB_COUNT(write_buffer));
    }


    void writeCommand(uint8_t command)
    {
        // GT911_COMMAND 0x8040
        uint8_t write_buffer[3] = {0x80, 0x40, command};
        writeBuffer(write_buffer, SENSORLIB_COUNT(write_buffer));
    }

    void inline clearBuffer()
    {
        writeGT911(GT911_POINT_INFO, 0x00);
    }

    bool probeAddress()
    {
        const uint8_t device_address[2]  = {GT911_SLAVE_ADDRESS_L, GT911_SLAVE_ADDRESS_H};
        for (size_t i = 0; i < SENSORLIB_COUNT(device_address); ++i) {
            __addr = device_address[i];
            for (int retry = 0; retry < 3; ++retry) {
                if (getChipID() == GT911_DEV_ID) {
                    return true;
                }
            }
        }
        return false;
    }


    bool initImpl()
    {
        int16_t x = 0, y = 0;

        if (__rst == SENSOR_PIN_NONE) {
            // Automatically determine the current device
            // address when using the reset pin without connection
            if (!probeAddress()) {
                return false;
            }
            log_i("Probe address is : 0x%X", __addr);

            // Reset Config
            reset();

            this->setGpioMode(__irq, INPUT);

        } else if (__addr == GT911_SLAVE_ADDRESS_H  &&
                   __rst != SENSOR_PIN_NONE &&
                   __irq != SENSOR_PIN_NONE) {

            log_i("GT911 using 0x28 address!");

            this->setGpioMode(__rst, OUTPUT);
            this->setGpioMode(__irq, OUTPUT);

            this->setGpioLevel(__rst, LOW);
            this->setGpioLevel(__irq, HIGH);
            delayMicroseconds(120);
            this->setGpioLevel(__rst, HIGH);
            delay(18);
            this->setGpioMode(__irq, INPUT);

        } else if (__addr == GT911_SLAVE_ADDRESS_L &&
                   __rst != SENSOR_PIN_NONE &&
                   __irq != SENSOR_PIN_NONE) {

            log_i("GT911 using 0xBA address!");

            this->setGpioMode(__rst, OUTPUT);
            this->setGpioMode(__irq, OUTPUT);

            this->setGpioLevel(__rst, LOW);
            this->setGpioLevel(__irq, LOW);
            delayMicroseconds(120);
            this->setGpioLevel(__rst, HIGH);
            delay(18);
            this->setGpioMode(__irq, INPUT);

        }

        // For variants where the GPIO is controlled by I2C, a delay is required here
        delay(20);

        __chipID = getChipID();
        log_i("Product id:%ld", __chipID);

        if (__chipID != GT911_DEV_ID) {
            log_i("Not find device GT911");
            return false;
        }
        log_i("Firmware version: 0x%x", getFwVersion());
        getResolution(&x, &y);
        log_i("Resolution : X = %d Y = %d", x, y);
        log_i("Vendor id:%d", getVendorID());
        log_i("Refresh Rate:%d ms", getRefreshRate());
        log_i("MaxTouchPoint:%d", getMaxTouchPoint());

        /*
        * For the ESP32 platform, the default buffer is 128.
        * Need to re-apply for a larger buffer to fully read the configuration table.
        * */
        if (!this->reallocBuffer(GT911_REG_LENGTH + 2 )) {
            log_e("realloc i2c buffer failed !");
            return false;
        }

        // Get the default interrupt trigger mode of the current screen
        getInterruptMode();

        if ( __irq_mode == RISING) {
            log_i("Interrupt Mode:  RISING");
        } else if (__irq_mode == FALLING) {
            log_i("Interrupt Mode:  FALLING");
        } else if (__irq_mode == LOW_LEVEL_QUERY) {
            log_i("Interrupt Mode:  LOW_LEVEL_QUERY");
        } else if (__irq_mode == HIGH_LEVEL_QUERY) {
            log_i("Interrupt Mode:  HIGH_LEVEL_QUERY");
        } else {
            log_e("UNKOWN");
        }


        return true;
    }

    int getReadMaskImpl()
    {
        return -1;
    }


protected:
    int __irq_mode;
};



