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
 * @file      TouchClassCST816.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#include "TouchClassCST816.h"

#define CST8xx_REG_STATUS            (0x00)
#define CST8xx_REG_XPOS_HIGH         (0x03)
#define CST8xx_REG_XPOS_LOW          (0x04)
#define CST8xx_REG_YPOS_HIGH         (0x05)
#define CST8xx_REG_YPOS_LOW          (0x06)
#define CST8xx_REG_DIS_AUTOSLEEP     (0xFE)
#define CST8xx_REG_CHIP_ID           (0xA7)
#define CST8xx_REG_FW_VERSION        (0xA9)
#define CST8xx_REG_SLEEP             (0xE5)

#define CST816S_CHIP_ID             (0xB4)
#define CST816T_CHIP_ID             (0xB5)
#define CST716_CHIP_ID              (0x20)
#define CST820_CHIP_ID              (0xB7)
#define CST816D_CHIP_ID             (0xB6)

#if defined(ARDUINO)
TouchClassCST816::TouchClassCST816():
    __center_btn_x(0),
    __center_btn_y(0)
{
}

bool TouchClassCST816::begin(PLATFORM_WIRE_TYPE &wire, uint8_t address, int sda, int scl)
{
    return SensorCommon::begin(wire, address, sda, scl);
}

#elif defined(ESP_PLATFORM)

#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
bool TouchClassCST816::begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr)
{
    return SensorCommon::begin(i2c_dev_bus_handle, addr);
}
#else
bool TouchClassCST816::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    return SensorCommon::begin(port_num, addr, sda, scl);
}

#endif //ESP_IDF_VERSION
#endif //ARDUINO

bool TouchClassCST816::begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
{
    return SensorCommon::begin(addr, readRegCallback, writeRegCallback);
}

void TouchClassCST816::reset()
{
    if (__rst != SENSOR_PIN_NONE) {
        this->setGpioMode(__rst, OUTPUT);
        this->setGpioLevel(__rst, LOW);
        delay(30);
        this->setGpioLevel(__rst, HIGH);
        delay(50);
    }
}

uint8_t TouchClassCST816::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    uint8_t buffer[13];
    if (readRegister(CST8xx_REG_STATUS, buffer, 13) == DEV_WIRE_ERR) {
        return 0;
    }

    if (!buffer[2] || !x_array || !y_array || !get_point) {
        return 0;
    }

    // Some CST816T will return all 0xFF after turning off automatic sleep.
    if (buffer[2] == 0xFF) {
        return 0;
    }

    // Only suitable for AMOLED 1.91 inch
    // RAW:00,00,01,80,78,02,58,00,00,FF,FF,FF,FF,
    // if (buffer[2] == 0x01 && buffer[3] == 0x80 && buffer[4] == 0x78 && buffer[5] == 0x02 && buffer[6] == 0x58) {
    //     if (__homeButtonCb) {
    //         __homeButtonCb(__userData);
    //     }
    //     return 0;
    // }

    uint8_t point = buffer[2] & 0x0F;

    // CST816 only supports single touch
    if (point > 1) {
        return 0;
    }

#ifdef LOG_PORT
    LOG_PORT.print("RAW:");
    for (int i = 0; i < 13; ++i) {
        LOG_PORT.printf("%02X,", buffer[i]);
    }
    LOG_PORT.println();
#endif

    int16_t tmp_x, tmp_y;

    tmp_x = ((buffer[CST8xx_REG_XPOS_HIGH] & 0x0F) << 8 | buffer[CST8xx_REG_XPOS_LOW]);
    tmp_y = ((buffer[CST8xx_REG_YPOS_HIGH] & 0x0F) << 8 | buffer[CST8xx_REG_YPOS_LOW]);

    // Depends on touch screen firmware
    if (tmp_x == __center_btn_x && tmp_y == __center_btn_y && __homeButtonCb) {
        __homeButtonCb(__userData);
        return 0;
    }

    // if (get_point == 2) {
    //     x_array[1] =  ((buffer[((uint8_t)0x09)] & 0x0F) << 8 | buffer[((uint8_t)0x10)]);
    //     y_array[1] =  ((buffer[((uint8_t)0x11)] & 0x0F) << 8 | buffer[((uint8_t)0x12)]);
    // }

    x_array[0] = tmp_x;
    y_array[0] = tmp_y;

#ifdef LOG_PORT
    for (int i = 0; i < point; i++) {
        LOG_PORT.printf("[%d] --> X:%d Y:%d \n", i, x_array[i], y_array[i]);
    }
#endif
    updateXY(point, x_array, y_array);

    return point;
}

bool TouchClassCST816::isPressed()
{
    static uint32_t lastPulse = 0;
    if (__irq != SENSOR_PIN_NONE) {
        int val = this->getGpioLevel(__irq) == LOW;
        if (val) {
            //Filter low levels with intervals greater than 1000ms
            val = (millis() - lastPulse > 1000) ?  false : true;
            lastPulse = millis();
            return val;
        }
        return false;
    }
    return getPoint(NULL, NULL, 1);
}


const char *TouchClassCST816::getModelName()
{
    switch (__chipID) {
    case CST816S_CHIP_ID:
        return "CST816S";
    case CST816T_CHIP_ID:
        return "CST816T";
    case CST716_CHIP_ID:
        return "CST716";
    case CST820_CHIP_ID:
        return "CST820";
    case CST816D_CHIP_ID:
        return "CST816D";
    default:
        break;
    }
    return "UNKONW";
}

void TouchClassCST816::sleep()
{
    writeRegister(CST8xx_REG_SLEEP, 0x03);
#ifdef ESP32
    if (__irq != SENSOR_PIN_NONE) {
        this->setGpioMode(__irq, OPEN_DRAIN);
    }
    if (__rst != SENSOR_PIN_NONE) {
        this->setGpioMode(__rst, OPEN_DRAIN);
    }
#endif
}

void TouchClassCST816::wakeup()
{
    reset();
}

void TouchClassCST816::idle()
{

}

uint8_t TouchClassCST816::getSupportTouchPoint()
{
    return 1;
}

bool TouchClassCST816::getResolution(int16_t *x, int16_t *y)
{
    return false;
}

void TouchClassCST816::setHomeButtonCallback(home_button_callback_t cb, void *user_data)
{
    __homeButtonCb = cb;
    __userData = user_data;
}

void TouchClassCST816::setCenterButtonCoordinate(int16_t x, int16_t y)
{
    __center_btn_x = x;
    __center_btn_y = y;
}


void TouchClassCST816::disableAutoSleep()
{
    switch (__chipID) {
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST820_CHIP_ID:
    case CST816D_CHIP_ID:
        reset();
        delay(50);
        writeRegister(CST8xx_REG_DIS_AUTOSLEEP, 0x01);
        break;
    case CST716_CHIP_ID:
    default:
        break;
    }
}

void TouchClassCST816::enableAutoSleep()
{
    switch (__chipID) {
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST820_CHIP_ID:
    case CST816D_CHIP_ID:
        reset();
        delay(50);
        writeRegister(CST8xx_REG_DIS_AUTOSLEEP, 0x00);
        break;
    case CST716_CHIP_ID:
    default:
        break;
    }
}

void TouchClassCST816::setGpioCallback(gpio_mode_fptr_t mode_cb,
                                       gpio_write_fptr_t write_cb,
                                       gpio_read_fptr_t read_cb)
{
    SensorCommon::setGpioModeCallback(mode_cb);
    SensorCommon::setGpioWriteCallback(write_cb);
    SensorCommon::setGpioReadCallback(read_cb);
}

bool TouchClassCST816::initImpl()
{

    if (__rst != SENSOR_PIN_NONE) {
        this->setGpioMode(__rst, OUTPUT);
    }

    if (__irq != SENSOR_PIN_NONE) {
        this->setGpioMode(__irq, INPUT);
    }

    reset();

    int chip_id =   readRegister(CST8xx_REG_CHIP_ID);
    log_i("Chip ID:0x%x\n", chip_id);

    int version =   readRegister(CST8xx_REG_FW_VERSION);
    log_i("Version :0x%x\n", version);

    // CST716  : 0x20
    // CST816S : 0xB4
    // CST816T : 0xB5
    // CST816D : 0xB6
    // CST226SE : A7 = 0X20
    if (chip_id != CST816S_CHIP_ID &&
            chip_id != CST816T_CHIP_ID  &&
            chip_id != CST820_CHIP_ID &&
            chip_id != CST816D_CHIP_ID &&
            (chip_id != CST716_CHIP_ID || version == 0)) {
        return false;
    }

    __chipID = chip_id;

    log_i("Touch type:%s\n", getModelName());

    return true;
}

int TouchClassCST816::getReadMaskImpl()
{
    return -1;
}





