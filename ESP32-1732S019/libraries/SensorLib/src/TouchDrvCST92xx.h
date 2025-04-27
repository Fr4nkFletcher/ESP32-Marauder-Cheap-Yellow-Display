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
 * @file      TouchDrvCST92xx.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-07
 */
#pragma once

#include "TouchDrvInterface.hpp"
#include "SensorCommon.tpp"
#include "SensorLib.h"
#include "REG/CST9xxConstants.h"


enum CST92_RunMode {
    CST92_MODE_NORMAL = (0x00),
    CST92_MODE_LOW_POWER = (0X01),
    CST92_MODE_DEEP_SLEEP = (0X02),
    CST92_MODE_WAKEUP = (0x03),
    CST92_MODE_DEBUG_DIFF = (0x04),
    CST92_MODE_DEBUG_RAWDATA = (0X05),
    CST92_MODE_FACTORY = (0x06),
    CST92_MODE_DEBUG_INFO = (0x07),
    CST92_MODE_UPDATE_FW = (0x08),
    CST92_MODE_FACTORY_HIGHDRV = (0x10),
    CST92_MODE_FACTORY_LOWDRV = (0x11),
    CST92_MODE_FACTORY_SHORT = (0x12),
    CST92_MODE_LPSCAN = (0x13),
};


class TouchDrvCST92xx : public TouchDrvInterface,
    public SensorCommon<TouchDrvCST92xx>
{
    friend class SensorCommon<TouchDrvCST92xx>;

public:

    TouchDrvCST92xx();


#if defined(ARDUINO)
    bool begin(PLATFORM_WIRE_TYPE &wire, uint8_t address, int sda, int scl);
#elif defined(ESP_PLATFORM)
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr);
#else
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl);
#endif //ESP_IDF_VERSION
#endif

    void jumpCheck();


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

    void setCoverScreenCallback(home_button_callback_t cb, void *user_data = NULL);

    void setGpioCallback(gpio_mode_fptr_t mode_cb,
                         gpio_write_fptr_t write_cb,
                         gpio_read_fptr_t read_cb);





private:

    typedef struct  {
        uint8_t finger_id;
        uint8_t evt;
        uint16_t x;
        uint16_t y;
    } cst9xx_point_t;

    bool setMode(uint8_t mode);
    bool enterBootloader();

    uint32_t readWordFromMem(uint8_t type, uint16_t mem_addr);
    void parseFingerData(uint8_t *data,  cst9xx_point_t *point);
    uint32_t get_u32_from_ptr(const void *ptr);


#if 0  /*DISABLE UPDATE FIRMWARE*/

    struct {
        bool firmware_info_ok;
        uint32_t firmware_ic_type;
        uint32_t firmware_version;
        uint32_t firmware_checksum;
        uint32_t firmware_project_id;
        uint8_t tx_num;
        uint8_t rx_num;
        uint8_t key_num;
    } IC_firmware;

    struct {
        bool ok;
        uint8_t *head_data;
        uint8_t *data;
        uint32_t checksum;
        uint32_t version;
        uint32_t project_id;
        uint32_t chip_type;
    } bin_data;

    bool getFirmwareInfo(void);
    int16_t eraseMem(void);
    int16_t writeSRAM(uint8_t *buf, uint16_t len);
    int16_t writeMemPage(uint16_t addr, uint8_t *buf, uint16_t len) ;
    int16_t writeMemAll(void) ;
    int16_t calculateVerifyChecksum(void) ;
    int16_t upgradeFirmware(void);
    uint32_t verifyFirmware(uint8_t *pdata, uint16_t order);
    int16_t parseFirmware(void);
    int16_t upgradeFirmwareJudge(void);
    int16_t getFirmwareAddress(uint8_t data_seq, uint16_t data_len) ;
    int16_t updateFirmware(void);
#endif


    uint32_t getChipType();
    bool initImpl();
    int getReadMaskImpl();

    uint16_t chipType;

protected:
    int16_t __center_btn_x;
    int16_t __center_btn_y;
    bool    __jump_check;
};







