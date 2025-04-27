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
 * @file      SensorBHI260AP.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once

#if defined(ARDUINO)

#include "bosch/BoschParse.h"
#include "bosch/SensorBhy2Define.h"
#include "bosch/firmware/BHI260AP.fw.h"




class SensorBHI260AP
{
    friend class BoschParse;
public:
    SensorBHI260AP(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = SENSORLIB_I2C_INTERFACE;
    }

    SensorBHI260AP(int cs, int mosi = -1, int miso = -1, int sck = -1,
                   PLATFORM_SPI_TYPE &spi = SPI
                  )
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = SENSORLIB_SPI_INTERFACE;
    }

    ~SensorBHI260AP()
    {
        deinit();
    }

    SensorBHI260AP()
    {
        memset(&__handler, 0, sizeof(__handler));
    }

    void setPins(int rst, int irq)
    {
        __handler.irq = irq;
        __handler.rst = rst;

    }

    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = SENSORLIB_I2C_INTERFACE;
        return initImpl();
    }

    bool init(
        PLATFORM_SPI_TYPE &spi,
        int cs, int mosi = MOSI, int miso = MISO, int sck = SCK)
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = SENSORLIB_SPI_INTERFACE;
        return initImpl();
    }

    bool init()
    {
        return initImpl();
    }

    void deinit()
    {
        if (__pro_buf) {
            free(__pro_buf);
        }
        __pro_buf = NULL;

        if (bhy2) {
            free(bhy2);
            bhy2 = NULL;
        }

        if (__handler.irq != SENSOR_PIN_NONE) {
            detachInterrupt(__handler.irq);
        }
        // end();
    }

    void reset()
    {
        if (__handler.rst != SENSOR_PIN_NONE) {
            digitalWrite(__handler.rst, HIGH);
            delay(5);
            digitalWrite(__handler.rst, LOW);
            delay(10);
            digitalWrite(__handler.rst, HIGH);
            delay(5);
        }
    }

    void update()
    {
        if (!__pro_buf) {
            return;
        }
        if (__handler.irq != SENSOR_PIN_NONE) {
            if (__data_available) {
                bhy2_get_and_process_fifo(__pro_buf, __pro_buf_size, bhy2);
            }
        } else {
            bhy2_get_and_process_fifo(__pro_buf, __pro_buf_size, bhy2);
        }
    }

    void setBootFormFlash(bool boot_from_flash)
    {
        __boot_from_flash = boot_from_flash;
    }


    bhy2_dev *getHandler()
    {
        return bhy2;
    }

    void printSensors(Stream &port)
    {
        uint8_t cnt = 0;
        bool presentBuff[256];

        for (uint16_t i = 0; i < sizeof(bhy2->present_buff); i++) {
            for (uint8_t j = 0; j < 8; j++) {
                presentBuff[i * 8 + j] = ((bhy2->present_buff[i] >> j) & 0x01);
            }
        }

        port.println("Present sensors: ");
        for (int i = 0; i < (int)sizeof(presentBuff); i++) {
            if (presentBuff[i]) {
                cnt++;
                port.print(i);
                port.print(" - ");
                port.print(get_sensor_name(i));
                port.println();
            }
        }
        port.printf("Total %u Sensor online .\n", cnt);
    }


    bool printInfo(Stream &stream)
    {
        uint16_t kernel_version = 0, user_version = 0;
        uint16_t rom_version = 0;
        uint8_t product_id = 0;
        uint8_t host_status = 0, feat_status = 0;
        uint8_t boot_status = 0;
        uint8_t sensor_error;
        struct bhy2_sensor_info info;

        /* Get product_id */
        __error_code = (bhy2_get_product_id(&product_id, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_product_id failed!", false);

        /* Get Kernel version */
        __error_code = (bhy2_get_kernel_version(&kernel_version, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_kernel_version failed!", false);

        /* Get User version */
        __error_code = (bhy2_get_user_version(&user_version, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_user_version failed!", false);

        /* Get ROM version */
        __error_code = (bhy2_get_rom_version(&rom_version, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_rom_version failed!", false);

        __error_code = (bhy2_get_host_status(&host_status, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_host_status failed!", false);

        __error_code = (bhy2_get_feature_status(&feat_status, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_feature_status failed!", false);

        stream.printf("Product ID     : %02x\n", product_id);
        stream.printf("Kernel version : %04u\n", kernel_version);
        stream.printf("User version   : %04u\n", user_version);
        stream.printf("ROM version    : %04u\n", rom_version);
        stream.printf("Power state    : %s\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
        stream.printf("Host interface : %s\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
        stream.printf("Feature status : 0x%02x\n", feat_status);

        /* Read boot status */
        __error_code = (bhy2_get_boot_status(&boot_status, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);

        stream.printf("Boot Status : 0x%02x: ", boot_status);

        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            stream.println("\tFlash detected. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            stream.println("\tFlash verify done. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
            stream.println("Flash verification failed. ");
        }

        if (boot_status & BHY2_BST_NO_FLASH) {
            stream.println("\tNo flash installed. ");
        }

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
            stream.println("\tHost interface ready. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            stream.println("\tFirmware verification done. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            stream.println("\tFirmware verification error. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_IDLE) {
            stream.println("\tFirmware halted. ");
        }

        /* Read error value */
        __error_code = (bhy2_get_error_value(&sensor_error, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_error_value failed!", false);
        if (sensor_error) {
            log_e("%s", get_sensor_error_text(sensor_error));
        }


        if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK) {

            bhy2_update_virtual_sensor_list(bhy2);

            /* Get present virtual sensor */
            bhy2_get_virt_sensor_list(bhy2);

            stream.printf("Virtual sensor list.\r\n");
            stream.printf("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\r\n");
            stream.printf("----------+--------------------------------------+-----+-----+-----------+-----------|\r\n");
            for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
                if (bhy2_is_sensor_available(i, bhy2)) {
                    if (i < BHY2_SENSOR_ID_CUSTOM_START) {
                        stream.printf(" %8u | %36s ", i, get_sensor_name(i));
                    }
                    __error_code = (bhy2_get_sensor_info(i, &info, bhy2));
                    BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_sensor_info failed!", false);
                    stream.printf("| %3u | %3u | %9.4f | %9.4f |\r\n",
                                  info.driver_id,
                                  info.driver_version,
                                  info.min_rate.f_val,
                                  info.max_rate.f_val);
                }
            }
        }
        return true;
    }

    bool setInterruptCtrl(uint8_t data)
    {
        __error_code = bhy2_set_host_interrupt_ctrl(data, bhy2);
        if (__error_code != BHY2_OK) {
            return false;
        }
    }

    uint8_t getInterruptCtrl()
    {
        uint8_t data;
        __error_code = bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (__error_code != BHY2_OK) {
            return 0;
        }
        return data;
    }

    void printInterruptCtrl(Stream &stream)
    {
        uint8_t data;
        __error_code = bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (__error_code != BHY2_OK) {
            return ;
        }
        stream.printf("Host interrupt control\r\n");
        stream.printf("-- Wake up FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
        stream.printf("-- Non wake up FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
        stream.printf("-- Status FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
        stream.printf("-- Debugging %s.\r\n", (data & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
        stream.printf("-- Fault %s.\r\n", (data & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
        stream.printf("-- Interrupt is %s.\r\n", (data & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
        stream.printf("-- Interrupt is %s triggered.\r\n", (data & BHY2_ICTL_EDGE) ? "pulse" : "level");
        stream.printf("-- Interrupt pin drive is %s.\r\n", (data & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
    }

    bool isReady()
    {
        uint8_t  boot_status = 0;
        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        log_i("boot_status:0x%x", boot_status);
        if (__error_code != BHY2_OK) {
            return false;
        }
        return (boot_status & BHY2_BST_HOST_INTERFACE_READY);
    }

    uint16_t getKernelVersion()
    {
        uint16_t version = 0;
        __error_code = bhy2_get_kernel_version(&version, bhy2);
        if ((__error_code != BHY2_OK) && (version == 0)) {
            return 0;
        }
        log_d("Boot successful. Kernel version %u.", version);
        return version;
    }



    void onEvent(BhySensorEvent event_id, BhyEventCb callback)
    {
        SensorEventCbList_t newEventHandler;
        newEventHandler.cb = callback;
        newEventHandler.event = event_id;
        BoschParse::bhyEventVector.push_back(newEventHandler);
    }

    void removeEvent(BhySensorEvent event_id, BhyEventCb callback)
    {
        if (!callback) {
            return;
        }
        for (uint32_t i = 0; i < BoschParse::bhyEventVector.size(); i++) {
            SensorEventCbList_t entry = BoschParse::bhyEventVector[i];
            if (entry.cb == callback && entry.event == event_id) {
                BoschParse::bhyEventVector.erase(BoschParse::bhyEventVector.begin() + i);
            }
        }
    }


    void onResultEvent(BhySensorID sensor_id, BhyParseDataCallback callback)
    {
        ParseCallBackList_t newEventHandler;
        newEventHandler.cb = callback;
        newEventHandler.id = sensor_id;
        BoschParse::bhyParseEventVector.push_back(newEventHandler);
    }

    void removeResultEvent(BhySensorID sensor_id, BhyParseDataCallback callback)
    {
        if (!callback) {
            return;
        }
        for (uint32_t i = 0; i < BoschParse::bhyParseEventVector.size(); i++) {
            ParseCallBackList_t entry = BoschParse::bhyParseEventVector[i];
            if (entry.cb == callback && entry.id == sensor_id) {
                BoschParse::bhyParseEventVector.erase(BoschParse::bhyParseEventVector.begin() + i);
            }
        }
    }

    void setProcessBufferSize(uint32_t size)
    {
        __pro_buf_size = size;
    }


    bool uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash = false)
    {
        uint8_t sensor_error;
        uint8_t boot_status;

        log_i("Upload Firmware ...");

        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);

        if (write2Flash) {
            if (boot_status & BHY2_BST_FLASH_DETECTED) {
                uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
                uint32_t end_addr = start_addr + length;
                log_i("Flash detected. Erasing flash to upload firmware");
                __error_code = bhy2_erase_flash(start_addr, end_addr, bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_erase_flash failed!", false);
            } else {
                log_e("Flash not detected");
                return false;
            }
            printf("Loading firmware into FLASH.");
            __error_code = bhy2_upload_firmware_to_flash(firmware, length, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_upload_firmware_to_flash failed!", false);
        } else {
            log_i("Loading firmware into RAM.");
            log_i("upload size = %lu", length);
            __error_code = bhy2_upload_firmware_to_ram(firmware, length, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_upload_firmware_to_ram failed!", false);
        }

        log_i("Loading firmware into RAM Done");
        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_error_value failed!", false);
        if (sensor_error != BHY2_OK) {
            __error_code = bhy2_get_error_value(&sensor_error, bhy2);
            log_e("%s", get_sensor_error_text(sensor_error));
            return false;
        }
        

        if (write2Flash) {
            log_i("Booting from FLASH.");
            __error_code = bhy2_boot_from_flash(bhy2);
        } else {
            log_i("Booting from RAM.");
            __error_code = bhy2_boot_from_ram(bhy2);
        }
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2 boot failed!", false);

        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        if (sensor_error) {
            log_e("%s", get_sensor_error_text(sensor_error));
            return false;
        }
        return sensor_error == BHY2_OK;
    }

    String getError()
    {
        String err = get_api_error(__error_code);
        err += " Code:" + String(__error_code);
        return err;
    }

    bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms)
    {
        if (!bhy2_is_sensor_available(sensor_id, bhy2)) {
            log_e("Sensor not present"); return false;
        }
        __error_code = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, report_latency_ms, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_virt_sensor_cfg failed!", false);
        log_i("Enable %s at %.2fHz.", get_sensor_name(sensor_id), sample_rate);
        return true;
    }

    struct bhy2_virt_sensor_conf getConfigure(uint8_t sensor_id)
    {
        bhy2_virt_sensor_conf conf;
        bhy2_get_virt_sensor_cfg(sensor_id, &conf, bhy2);
        log_i("range:%u sample_rate:%f latency:%lu sensitivity:%u\n", conf.range, conf.sample_rate, conf.latency, conf.sensitivity);
        return conf;
    }

    float getScaling(uint8_t sensor_id)
    {
        return get_sensor_default_scaling(sensor_id);
    }

    void setFirmware(const uint8_t *image, size_t image_len, bool write_flash)
    {
        __firmware = image;
        __firmware_size = image_len;
        __write_flash = write_flash;
    }

    static const char *getSensorName(uint8_t sensor_id)
    {
        return get_sensor_name(sensor_id);
    }

    // Get an accuracy report
    uint8_t getAccuracy()
    {
        return __accuracy;
    }

private:

    bool bootFromFlash()
    {
        int8_t rslt;
        uint8_t boot_status, feat_status;
        uint8_t error_val = 0;
        uint16_t tries = 300; /* Wait for up to little over 3s */

        log_d("Waiting for firmware verification to complete");
        do {
            __error_code = bhy2_get_boot_status(&boot_status, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
            if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
                break;
            }
            delay(10);
        } while (tries--);

        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
        print_boot_status(boot_status);

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {

            if (boot_status & BHY2_BST_FLASH_DETECTED) {

                /* If no firmware is running, boot from Flash */
                log_d("Booting from flash");
                rslt = bhy2_boot_from_flash(bhy2);
                if (rslt != BHY2_OK) {
                    log_e("%s. Booting from flash failed.\r\n", get_api_error(rslt));
                    __error_code = bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_val, 1, bhy2);
                    BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_regs failed!", false);
                    if (error_val) {
                        log_e("%s\r\n", get_sensor_error_text(error_val));
                    }
                    return false;
                }

                __error_code = bhy2_get_boot_status(&boot_status, bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
                print_boot_status(boot_status);

                if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
                    /* hub is not ready, need reset hub */
                    log_d("Host interface is not ready, triggering a reset");
                    __error_code = bhy2_soft_reset(bhy2);
                    BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_soft_reset failed!", false);
                }

                __error_code = (bhy2_get_feature_status(&feat_status, bhy2));
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "Reading Feature status failed, booting from flash failed!", false);

            } else {
                log_e("Can't detect external flash");
                return false;
            }
        } else {
            log_e("Host interface is not ready");
            return false;
        }

        log_d("Booting from flash successful");
        return true;
    }



    void print_boot_status(uint8_t boot_status)
    {
        log_d("Boot Status : 0x%02x: ", boot_status);
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            log_d("Flash detected. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            log_d("Flash verify done. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
            log_d("Flash verification failed. ");
        }

        if (boot_status & BHY2_BST_NO_FLASH) {
            log_d("No flash installed. ");
        }

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
            log_d("Host interface ready. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            log_d("Firmware verification done. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            log_d("Firmware verification error. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_IDLE) {
            log_d("Firmware halted. ");
        }
    }

    static void handleISR()
    {
        __data_available = true;
    }


    bool initImpl()
    {
        uint8_t product_id = 0;

        if (__handler.rst != SENSOR_PIN_NONE) {
            pinMode(__handler.rst, OUTPUT);
        }

        reset();

        bhy2 = (struct bhy2_dev *)malloc(sizeof(struct bhy2_dev ));
        BHY2_RLST_CHECK(!bhy2, " Device handler malloc failed!", false);

        switch (__handler.intf) {
        case BHY2_I2C_INTERFACE:
            // esp32s3 test I2C maximum read and write is 64 bytes
            __max_rw_length = 64;
            BHY2_RLST_CHECK(!__handler.u.i2c_dev.wire, "Wire ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(__handler)) {
                log_e("setup_interfaces failed");
                return false;
            }
            __error_code = bhy2_init(BHY2_I2C_INTERFACE,
                                     SensorInterfaces::bhy2_i2c_read,
                                     SensorInterfaces::bhy2_i2c_write,
                                     SensorInterfaces::bhy2_delay_us,
                                     __max_rw_length, &__handler, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_init failed!", false);
            // __error_code = bhy2_set_host_intf_ctrl(BHY2_I2C_INTERFACE, bhy2);
            // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_host_intf_ctrl failed!", false);
            break;

        case BHY2_SPI_INTERFACE:
            // esp32s3 test SPI maximum read and write is 256 bytes
            __max_rw_length = 256;
            BHY2_RLST_CHECK(!__handler.u.spi_dev.spi, "SPI ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(__handler)) {
                log_e("setup_interfaces failed");
                return false;
            }
            __error_code = bhy2_init(BHY2_SPI_INTERFACE,
                                     SensorInterfaces::bhy2_spi_read,
                                     SensorInterfaces::bhy2_spi_write,
                                     SensorInterfaces::bhy2_delay_us,
                                     __max_rw_length,
                                     &__handler,
                                     bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_init failed!", false);
            // __error_code = bhy2_set_host_intf_ctrl(BHY2_SPI_INTERFACE, bhy2);
            // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_host_intf_ctrl failed!", false);
            break;
        default:
            return false;
        }

        __error_code = bhy2_soft_reset(bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "reset bhy2 failed!", false);

        __error_code = bhy2_get_product_id(&product_id, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_product_id failed!", false);


        /* Check for a valid product ID */
        if (product_id != BHY2_PRODUCT_ID) {
            log_e("Product ID read %X. Expected %X", product_id, BHY2_PRODUCT_ID);
            return false;
        } else {
            log_i("BHI260/BHA260 found. Product ID read %X", product_id);
        }

        if (!__firmware) {
            // Default write to ram
            setFirmware(bhy2_firmware_image, sizeof(bhy2_firmware_image), false);
        }

        if (__boot_from_flash) {
            if (!bootFromFlash()) {
                //** If the boot from flash fails, re-upload the firmware to flash
                __error_code = bhy2_soft_reset(bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "reset bhy2 failed!", false);

                if (!uploadFirmware(__firmware, __firmware_size, __write_flash)) {
                    log_e("uploadFirmware failed!");
                    return false;
                }
            }
        } else {
            // ** Upload firmware to RAM
            if (!uploadFirmware(__firmware, __firmware_size, __write_flash)) {
                log_e("uploadFirmware failed!");
                return false;
            }
        }

        uint16_t version = getKernelVersion();
        BHY2_RLST_CHECK(!version, "getKernelVersion failed!", false);
        log_i("Boot successful. Kernel version %u.", version);

        //Set event callback
        __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, BoschParse::parseMetaEvent, (void *)&__accuracy, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);

        __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, BoschParse::parseMetaEvent, (void *)&__accuracy, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);

        // __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, BoschParse::parseDebugMessage, NULL, bhy2);
        // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback parseDebugMessage failed!", false);

        //Set process buffer
#if     (defined(ESP32) || defined(ARDUINO_ARCH_ESP32)) && defined(BOARD_HAS_PSRAM)
        __pro_buf = (uint8_t *)ps_malloc(__pro_buf_size);
#else
        __pro_buf = (uint8_t *)malloc(__pro_buf_size);
#endif
        BHY2_RLST_CHECK(!__pro_buf, "process buffer malloc failed!", false);

        __error_code = bhy2_get_and_process_fifo(__pro_buf, __pro_buf_size, bhy2);
        if (__error_code != BHY2_OK) {
            log_e("bhy2_get_and_process_fifo failed");
            free(__pro_buf);
            return false;
        }

        /* Update the callback table to enable parsing of sensor hintr_ctrl */
        bhy2_update_virtual_sensor_list(bhy2);

        /* Get present virtual sensor */
        bhy2_get_virt_sensor_list(bhy2);

        // Only register valid sensor IDs
        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
            if (bhy2_is_sensor_available(i, bhy2)) {
                bhy2_register_fifo_parse_callback(i, BoschParse::parseData, NULL, bhy2);
            }
        }

        if (__handler.irq != SENSOR_PIN_NONE) {
#if defined(ARDUINO_ARCH_RP2040)
            attachInterrupt((pin_size_t)(__handler.irq), handleISR, (PinStatus )RISING);
#elif defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32) 
            attachInterrupt(__handler.irq, handleISR, RISING);
#else
#error "Interrupt registration not implemented"
#endif
        }

        return __error_code == BHY2_OK;
    }

protected:
    struct bhy2_dev  *bhy2 = NULL;
    SensorLibConfigure __handler;
    int8_t           __error_code;
    static volatile bool __data_available;
    uint8_t          *__pro_buf = NULL;
    size_t           __pro_buf_size = BHY_PROCESS_BUFFER_SIZE;
    const uint8_t    *__firmware;
    size_t          __firmware_size;
    bool            __write_flash;
    bool            __boot_from_flash;
    uint16_t        __max_rw_length;
    uint8_t         __accuracy;      /* Accuracy is reported as a meta event. */
};


#endif /*defined(ARDUINO)*/




