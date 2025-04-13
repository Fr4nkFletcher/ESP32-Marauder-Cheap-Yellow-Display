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
 * @file      TouchDrvFT6X36.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-01
 *
 */
#pragma once

#include "REG/FT6X36Constants.h"
#include "TouchDrvInterface.hpp"
#include "SensorCommon.tpp"

class TouchDrvFT6X36 :
    public TouchDrvInterface,
    public SensorCommon<TouchDrvFT6X36>
{
    friend class SensorCommon<TouchDrvFT6X36>;
public:


    enum GesTrue {
        NO_GESTURE,
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_RIGHT,
        ZOOM_IN,
        ZOOM_OUT,
    } ;

    enum EventFlag {
        EVENT_PUT_DOWN,
        EVENT_PUT_UP,
        EVENT_CONTACT,
        EVENT_NONE,
    } ;

    enum PowerMode {
        PMODE_ACTIVE = 0,         // ~4mA
        PMODE_MONITOR = 1,        // ~3mA
        PMODE_DEEPSLEEP = 3,      // ~100uA  The reset pin must be pulled down to wake up
    } ;

    EventFlag event;

#if defined(ARDUINO)
    TouchDrvFT6X36(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = FT6X36_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    TouchDrvFT6X36()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = FT6X36_SLAVE_ADDRESS;
    }

    ~TouchDrvFT6X36()
    {
        deinit();
    }

#if defined(ARDUINO)

    bool begin(PLATFORM_WIRE_TYPE &w,
               uint8_t addr = FT6X36_SLAVE_ADDRESS,
               int sda = DEFAULT_SDA,
               int scl = DEFAULT_SCL)
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

    uint8_t getDeviceMode(void)
    {
        return readRegister(FT6X36_REG_MODE) & 0x03;
    }

    // Obtaining gestures depends on whether the built-in firmware of the chip has this function
    uint8_t getGesture()
    {
        int val = readRegister(FT6X36_REG_GEST);
        switch (val) {
        case 0x10:
            return MOVE_UP;
        case 0x14:
            return MOVE_RIGHT;
        case 0x18:
            return MOVE_DOWN;
        case 0x1C:
            return MOVE_LEFT;
        case 0x48:
            return ZOOM_IN;
        case 0x49:
            return ZOOM_OUT;
        default:
            break;
        }
        return NO_GESTURE;
    }

    void setThreshold(uint8_t value)
    {
        writeRegister(FT6X36_REG_THRESHOLD, value);
    }

    uint8_t getThreshold(void)
    {
        return readRegister(FT6X36_REG_THRESHOLD);
    }

    uint8_t getMonitorTime(void)
    {
        return readRegister(FT6X36_REG_MONITOR_TIME);
    }

    void setMonitorTime(uint8_t sec)
    {
        writeRegister(FT6X36_REG_MONITOR_TIME, sec);
    }

    // Calibration useless actually,
    // any value  set will not be valid,
    // depending on the internal firmware of the chip.
    /*
    void enableAutoCalibration(void)
    {
        writeRegister(FT6X36_REG_AUTO_CLB_MODE, 0x00);
    }

    void disableAutoCalibration(void)
    {
        writeRegister(FT6X36_REG_AUTO_CLB_MODE, 0xFF);
    }
    */

    uint16_t getLibraryVersion()
    {
        uint8_t buffer[2];
        readRegister(FT6X36_REG_LIB_VERSION_H, buffer, 2);
        return (buffer[0] << 8) | buffer[1];
    }

    // The interrupt is triggered only if a touch is detected during the scan cycle
    void interruptPolling(void)
    {
        //datasheet this bit is 0,Actually, it's wrong
        writeRegister(FT6X36_REG_INT_STATUS, 1);
    }

    // Triggers an interrupt whenever a touch is detected
    void interruptTrigger(void)
    {
        //datasheet this bit is 1,Actually, it's wrong
        writeRegister(FT6X36_REG_INT_STATUS, 0);
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1)
    {
        uint8_t buffer[16];

        if (!x_array || !y_array || !size)
            return 0;

        if (readRegister(FT6X36_REG_MODE, buffer, 16) == DEV_WIRE_ERR) {
            return 0;
        }

        // uint8_t mode = buffer[0];
        //REG 0x01
        // uint8_t gesture = buffer[1];
        //REG 0x02
        uint8_t point = buffer[2] & 0x0F;
        if (point == 0 || point == 0x0F) {
            return 0;
        }

        //REG 0x03 ~ 0x04
        // uint8_t eventFlag = (buffer[3] & 0xC0) >> 6;
        uint16_t posX = ((buffer[3] & 0x0F) << 8) | buffer[4];
        //REG 0x05 ~ 0x06
        uint16_t posY = ((buffer[5] & 0x0F) << 8) | buffer[6] ;


        x_array[0] = posX;
        y_array[0] = posY;

#ifdef LOG_PORT
        LOG_PORT.println("----------------------------------------------------------------------------");
        LOG_PORT.println("Touched Gesture EvenFlag    [0]PosX    [0]PosY  [1]PosX    [1]PosY");
        LOG_PORT.print(point); LOG_PORT.print("\t");
        // LOG_PORT.print(gesture); LOG_PORT.print("\t");
        // LOG_PORT.print(eventFlag); LOG_PORT.print("\t");
        LOG_PORT.print(posX); LOG_PORT.print("\t");
        LOG_PORT.print(posY); LOG_PORT.print("\t");
#endif

        if (point == 2) {
            //REG 0x09 ~ 0x0A
            posX = ((buffer[9] & 0x0F) << 8) | buffer[10];
            //REG 0x0B ~ 0x0C
            posY = ((buffer[11] & 0x0F) << 8) | buffer[12] ;

#ifdef LOG_PORT
            LOG_PORT.print(posX); LOG_PORT.print("\t");
            LOG_PORT.print(posY); LOG_PORT.print("\t");
#endif

            if (size == 2) {
                x_array[1] = posX;
                y_array[1] = posY;
            }
        }
#ifdef LOG_PORT
        LOG_PORT.println();
#endif
        updateXY(point, x_array, y_array);

        return point;
    }

    bool isPressed()
    {
        if (__irq != SENSOR_PIN_NONE) {
            return this->getGpioLevel(__irq) == LOW;
        }
        return readRegister(FT6X36_REG_STATUS) & 0x0F;
    }

    void setPowerMode(PowerMode mode)
    {
        writeRegister(FT6X36_REG_POWER_MODE, mode);
    }

    void sleep()
    {
        writeRegister(FT6X36_REG_POWER_MODE, PMODE_DEEPSLEEP);
    }

    void wakeup()
    {
        reset();
    }

    void idle()
    {

    }

    uint8_t getSupportTouchPoint()
    {
        return 1;
    }

    uint32_t getChipID(void)
    {
        return readRegister(FT6X36_REG_CHIP_ID);
    }

    uint8_t getVendorID(void)
    {
        return readRegister(FT6X36_REG_VENDOR1_ID);
    }

    uint8_t getErrorCode(void)
    {
        return readRegister(FT6X36_REG_ERROR_STATUS);
    }

    const char *getModelName()
    {
        switch (__chipID) {
        case FT6206_CHIP_ID: return "FT6206";
        case FT6236_CHIP_ID: return "FT6236";
        case FT6236U_CHIP_ID: return "FT6236U";
        case FT3267_CHIP_ID: return "FT3267";
        default: return "UNKNOWN";
        }
    }


    bool getResolution(int16_t *x, int16_t *y)
    {
        return false;
    }

    void reset()
    {
        if (__rst != SENSOR_PIN_NONE) {
            this->setGpioMode(__rst, OUTPUT);
            this->setGpioLevel(__rst, HIGH);
            delay(10);
            this->setGpioLevel(__rst, LOW);
            delay(30);
            this->setGpioLevel(__rst, HIGH);
            // For the variant of GPIO extended RST,
            // communication and delay are carried out simultaneously, and 160ms is measured in T-RGB esp-idf new api
            delay(160);
        }
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
    bool initImpl()
    {
        if (__irq != SENSOR_PIN_NONE) {
            this->setGpioMode(__irq, INPUT);
        }

        reset();

        uint8_t vendId = readRegister(FT6X36_REG_VENDOR1_ID);


        if (vendId != FT6X36_VEND_ID) {
            log_e("Vendor id is 0x%X not match!", vendId);
            return false;
        }

        __chipID = readRegister(FT6X36_REG_CHIP_ID);

        if ((__chipID != FT6206_CHIP_ID) &&
                (__chipID != FT6236_CHIP_ID) &&
                (__chipID != FT6236U_CHIP_ID)  &&
                (__chipID != FT3267_CHIP_ID)
           ) {
            log_e("Vendor id is not match!");
            log_e("ChipID:0x%lx should be 0x06 or 0x36 or 0x64", __chipID);
            return false;
        }

        log_i("Vend ID: 0x%X", vendId);
        log_i("Chip ID: 0x%lx", __chipID);
        log_i("Firm Version: 0x%X", readRegister(FT6X36_REG_FIRM_VERS));
        log_i("Point Rate Hz: %u", readRegister(FT6X36_REG_PERIOD_ACTIVE));
        log_i("Thresh : %u", readRegister(FT6X36_REG_THRESHOLD));

        // change threshold to be higher/lower
        writeRegister(FT6X36_REG_THRESHOLD, 60);

        log_i("Chip library version : 0x%x", getLibraryVersion());

        // This register describes period of monitor status, it should not less than 30.
        log_i("Chip period of monitor status : 0x%x", readRegister(FT6X36_REG_PERIOD_MONITOR));

        // This register describes the period of active status, it should not less than 12


        return true;
    }

    int getReadMaskImpl()
    {
        return -1;
    }

};



