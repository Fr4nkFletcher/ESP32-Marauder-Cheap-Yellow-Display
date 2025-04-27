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
 * @file      SensorQMI8658.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#pragma once

#include "REG/QMI8658Constants.h"
#include "SensorCommon.tpp"
#ifndef ARDUINO
#include <math.h>
#include <stdio.h>
#endif
typedef struct __IMUdata {
    float x;
    float y;
    float z;
} IMUdata;

class SensorQMI8658 :
    public SensorCommon<SensorQMI8658>
{
    friend class SensorCommon<SensorQMI8658>;
public:

    typedef void (*EventCallBack_t)(void);

    enum AccelRange {
        ACC_RANGE_2G,
        ACC_RANGE_4G,
        ACC_RANGE_8G,
        ACC_RANGE_16G
    };

    enum GyroRange {
        GYR_RANGE_16DPS,
        GYR_RANGE_32DPS,
        GYR_RANGE_64DPS,
        GYR_RANGE_128DPS,
        GYR_RANGE_256DPS,
        GYR_RANGE_512DPS,
        GYR_RANGE_1024DPS,
    };

    // In 6DOF mode (accelerometer and gyroscope are both enabled),
    // the output data rate is derived from the nature frequency of gyroscope
    enum AccelODR {
        ACC_ODR_1000Hz = 3,
        ACC_ODR_500Hz,
        ACC_ODR_250Hz,
        ACC_ODR_125Hz,
        ACC_ODR_62_5Hz,
        ACC_ODR_31_25Hz,
        ACC_ODR_LOWPOWER_128Hz  = 12,   //The accelerometer low power mode is only available when the gyroscope is disabled
        ACC_ODR_LOWPOWER_21Hz,          //The accelerometer low power mode is only available when the gyroscope is disabled
        ACC_ODR_LOWPOWER_11Hz,          //The accelerometer low power mode is only available when the gyroscope is disabled
        ACC_ODR_LOWPOWER_3Hz            //The accelerometer low power mode is only available when the gyroscope is disabled
    };

    enum GyroODR {
        GYR_ODR_7174_4Hz,
        GYR_ODR_3587_2Hz,
        GYR_ODR_1793_6Hz,
        GYR_ODR_896_8Hz,
        GYR_ODR_448_4Hz,
        GYR_ODR_224_2Hz,
        GYR_ODR_112_1Hz,
        GYR_ODR_56_05Hz,
        GYR_ODR_28_025Hz
    };

    enum TapEvent {
        INVALID_TAP,
        SINGLE_TAP,
        DOUBLE_TAP,
    };

    //Low-Pass Filter.
    enum LpfMode {
        LPF_MODE_0,     //2.66% of ODR
        LPF_MODE_1,     //3.63% of ODR
        LPF_MODE_2,     //5.39% of ODR
        LPF_MODE_3,     //13.37% of ODR
        LPF_OFF,        //OFF Low-Pass Filter
    };

    enum MotionEvent {
        MOTION_TAP,
        MOTION_ANT_MOTION,
        MOTION_NO_MOTION,
        MOTION_SIGNIFICANT,
        MOTION_PEDOMETER,
    };

    enum IntPin {
        INTERRUPT_PIN_1,
        INTERRUPT_PIN_2,
        INTERRUPT_PIN_DISABLE
    };

    enum FIFO_Samples {
        FIFO_SAMPLES_16,
        FIFO_SAMPLES_32,
        FIFO_SAMPLES_64,
        FIFO_SAMPLES_128,
    } ;

    enum FIFO_Mode {
        // Configure the FIFO_MODE to ‘Bypass’ (0) mode, will disable the FIFO functionality.
        FIFO_MODE_BYPASS,
        // In ‘FIFO’ mode, once FIFO is full,
        // the data filling will stop and new data will be discarded until host reads out the FIFO data and release the space for new data to be written to.
        FIFO_MODE_FIFO,
        // In ‘Stream’ mode, once FIFO is full,
        // the data filling will continue and the oldest data will be discarded,
        // until host reads out the FIFO data and release the space for new data to be written to
        FIFO_MODE_STREAM,
    };

    enum SampleMode {
        SYNC_MODE,      //Synchronous sampling
        ASYNC_MODE,     //Asynchronous sampling
    };

    enum CommandTable {
        CTRL_CMD_ACK                            = 0x00,
        CTRL_CMD_RST_FIFO                       = 0x04,
        CTRL_CMD_REQ_FIFO                       = 0x05,
        CTRL_CMD_WRITE_WOM_SETTING              = 0x08,
        CTRL_CMD_ACCEL_HOST_DELTA_OFFSET        = 0x09,
        CTRL_CMD_GYRO_HOST_DELTA_OFFSET         = 0x0A,
        CTRL_CMD_CONFIGURE_TAP                  = 0x0C,
        CTRL_CMD_CONFIGURE_PEDOMETER            = 0x0D,
        CTRL_CMD_CONFIGURE_MOTION               = 0x0E,
        CTRL_CMD_RESET_PEDOMETER                = 0x0F,
        CTRL_CMD_COPY_USID                      = 0x10,
        CTRL_CMD_SET_RPU                        = 0x11,
        CTRL_CMD_AHB_CLOCK_GATING               = 0x12,
        CTRL_CMD_ON_DEMAND_CALIBRATION          = 0xA2,
        CTRL_CMD_APPLY_GYRO_GAINS               = 0xAA,
    };

    enum StatusReg {
        EVENT_SIGNIFICANT_MOTION = 128,
        EVENT_NO_MOTION = 64,
        EVENT_ANY_MOTION = 32,
        EVENT_PEDOMETER_MOTION = 16,
        EVENT_WOM_MOTION = 4,
        EVENT_TAP_MOTION = 2,
    };

    enum SensorStatus {
        STATUS_INT_CTRL9_CMD_DONE = _BV(0),
        STATUS_INT_LOCKED = _BV(1),
        STATUS_INT_AVAIL = _BV(2),
        STATUS0_GYRO_DATA_READY = _BV(3),
        STATUS0_ACCEL_DATA_READY = _BV(4),
        STATUS1_SIGNIFICANT_MOTION = _BV(5),
        STATUS1_NO_MOTION = _BV(6),
        STATUS1_ANY_MOTION = _BV(7),
        STATUS1_PEDOMETER_MOTION = _BV(8),
        STATUS1_WOM_MOTION = _BV(9),
        STATUS1_TAP_MOTION = _BV(10),
    };

    enum TapDetectionPriority {
        PRIORITY0,      // (X > Y> Z)
        PRIORITY1,      // (X > Z > Y)
        PRIORITY2,      // (Y > X > Z)
        PRIORITY3,      // (Y > Z > X)
        PRIORITY4,      // (Z > X > Y)
        PRIORITY5,      // (Z > Y > X)
    };

    enum MotionCtrl {

        ANY_MOTION_EN_X = _BV(0),
        ANY_MOTION_EN_Y = _BV(1),
        ANY_MOTION_EN_Z = _BV(2),

        //Logic-AND between events of enabled axes for No-Motion detection, Otherwise, logical OR
        ANY_MOTION_LOGIC_AND = _BV(3),

        NO_MOTION_EN_X = _BV(4),
        NO_MOTION_EN_Y = _BV(5),
        NO_MOTION_EN_Z = _BV(6),

        //Logic-AND between events of enabled axes for No-Motion detection , Otherwise, logical OR
        NO_MOTION_LOGIC_OR = _BV(7),
    };

#if defined(ARDUINO)
    SensorQMI8658(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = QMI8658_L_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }

    SensorQMI8658(int cs, int mosi = -1, int miso = -1, int sck = -1, PLATFORM_SPI_TYPE &spi = SPI)
    {
        __spi = &spi;
        __cs = cs;
        __miso = miso;
        __mosi = mosi;
        __sck = sck;
    }

#endif

    SensorQMI8658()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = QMI8658_L_SLAVE_ADDRESS;
    }

    ~SensorQMI8658()
    {
        log_i("~SensorQMI8658");
        if (__fifo_buffer) {
            free(__fifo_buffer);
            __fifo_buffer = NULL;
        }
        deinit();
    }

#if defined(ARDUINO)
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = QMI8658_L_SLAVE_ADDRESS)
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }
#endif

    void deinit()
    {
        // end();
    }

    bool reset(bool waitResult = true, uint32_t timeout = 500)
    {
        int val = 0;  // initialize with some value to avoid compilation errors
        writeRegister(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
        // Maximum 15ms for the Reset process to be finished
        if (waitResult) {
            uint32_t start = millis();
            while (millis() - start < timeout) {
                val = readRegister(QMI8658_REG_RST_RESULT);
                if (val != DEV_WIRE_ERR && val == QMI8658_REG_RST_RESULT_VAL) {

                    //EN.ADDR_AI
                    setRegisterBit(QMI8658_REG_CTRL1, 6);

                    return true;
                }
                delay(10);
            }
            log_e("Reset chip failed, Response val = %d - 0x%X", val, val);
            return false;
        }

        //EN.ADDR_AI
        setRegisterBit(QMI8658_REG_CTRL1, 6);

        return true;
    }

    uint8_t getChipID()
    {
        return readRegister(QMI8658_REG_REVISION);
    }

    int whoAmI()
    {
        return readRegister(QMI8658_REG_WHOAMI);
    }

    uint32_t getTimestamp()
    {
        uint8_t  buffer[3];
        uint32_t timestamp;
        if (readRegister(QMI8658_REG_TIMESTAMP_L, buffer, 3) != DEV_WIRE_ERR) {
            timestamp = (uint32_t)(((uint32_t)buffer[2] << 16) |
                                   ((uint32_t)buffer[1] << 8) | buffer[0]);
            if (timestamp > lastTimestamp) {
                lastTimestamp = timestamp;
            } else {
                lastTimestamp = (timestamp + 0x1000000 - lastTimestamp);
            }
        }
        return lastTimestamp;
    }


    float getTemperature_C()
    {
        uint8_t buffer[2];
        if (readRegister(QMI8658_REG_TEMPERATURE_L, buffer, 2) !=  DEV_WIRE_ERR) {
            return (float)buffer[1] + ((float)buffer[0] / 256.0);
        }
        return NAN;
    }

    void enableINT(IntPin pin, bool enable = true)
    {
        switch (pin) {
        case INTERRUPT_PIN_1:
            enable ? setRegisterBit(QMI8658_REG_CTRL1, 3) : clrRegisterBit(QMI8658_REG_CTRL1, 3);
            __irq_enable_mask = enable ? __irq_enable_mask | 0x01 : __irq_enable_mask & 0xFE;
            break;
        case INTERRUPT_PIN_2:
            enable ? setRegisterBit(QMI8658_REG_CTRL1, 4) : clrRegisterBit(QMI8658_REG_CTRL1, 4);
            __irq_enable_mask = enable ? __irq_enable_mask | 0x02 : __irq_enable_mask & 0xFD;
            break;
        default:
            break;
        }
    }

    uint8_t getIrqStatus()
    {
        return readRegister(QMI8658_REG_STATUS_INT);
    }


    void enableDataReadyINT(bool enable = true)
    {
        enable ? clrRegisterBit(QMI8658_REG_CTRL7, 5) :
        setRegisterBit(QMI8658_REG_CTRL7, 5);
    }


    int configAccelerometer(AccelRange range, AccelODR odr, LpfMode lpfOdr = LPF_MODE_0)
    {
        bool en = isEnableAccelerometer();

        if (en) {
            disableAccelerometer();
        }

        //setAccelRange
        if (writeRegister(QMI8658_REG_CTRL2, 0x8F, (range << 4)) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        switch (range) {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that
        // 2-bit value:
        case ACC_RANGE_2G:  accelScales = 2.0 / 32768.0; break;
        case ACC_RANGE_4G:  accelScales = 4.0 / 32768.0; break;
        case ACC_RANGE_8G:  accelScales = 8.0 / 32768.0; break;
        case ACC_RANGE_16G: accelScales = 16.0 / 32768.0; break;
        }

        // setAccelOutputDataRate
        if (writeRegister(QMI8658_REG_CTRL2, 0xF0, odr) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        if (lpfOdr != LPF_OFF) {
            // setAccelLowPassFitterOdr
            if (writeRegister(QMI8658_REG_CTRL5, QMI8658_ACCEL_LPF_MASK,  (lpfOdr << 1)) != DEV_WIRE_NONE) {
                return DEV_WIRE_ERR;
            }
            // Enable Low-Pass Fitter
            setRegisterBit(QMI8658_REG_CTRL5, 0);
        } else {
            // Disable Low-Pass Fitter
            clrRegisterBit(QMI8658_REG_CTRL5, 0);
        }

        // setAccelSelfTest
        // selfTest ? setRegisterBit(QMI8658_REG_CTRL2, 7) : clrRegisterBit(QMI8658_REG_CTRL2, 7);

        if (en) {
            enableAccelerometer();
        }

        return DEV_WIRE_NONE;
    }


    int configGyroscope(GyroRange range, GyroODR odr, LpfMode lpfOdr = LPF_MODE_0)
    {
        bool en = isEnableGyroscope();

        if (en) {
            disableGyroscope();
        }

        // setGyroRange
        if (writeRegister(QMI8658_REG_CTRL3, 0x8F, (range << 4)) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        switch (range) {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that
        // 2-bit value:
        case GYR_RANGE_16DPS: gyroScales = 16.0 / 32768.0; break;
        case GYR_RANGE_32DPS: gyroScales = 32.0 / 32768.0; break;
        case GYR_RANGE_64DPS: gyroScales = 64.0 / 32768.0; break;
        case GYR_RANGE_128DPS: gyroScales = 128.0 / 32768.0; break;
        case GYR_RANGE_256DPS: gyroScales = 256.0 / 32768.0; break;
        case GYR_RANGE_512DPS: gyroScales = 512.0 / 32768.0; break;
        case GYR_RANGE_1024DPS: gyroScales = 1024.0 / 32768.0; break;
        }

        // setGyroOutputDataRate
        if (writeRegister(QMI8658_REG_CTRL3, 0xF0, odr) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        // setGyroLowPassFitterOdr
        if (lpfOdr != LPF_OFF) {
            if (writeRegister(QMI8658_REG_CTRL5, QMI8658_GYRO_LPF_MASK,  (lpfOdr << 5)) != DEV_WIRE_NONE) {
                return DEV_WIRE_ERR;
            }
            // Enable Low-Pass Fitter
            setRegisterBit(QMI8658_REG_CTRL5, 4);
        } else {
            // Disable Low-Pass Fitter
            clrRegisterBit(QMI8658_REG_CTRL5, 4);
        }

        // setGyroSelfTest
        // selfTest ? setRegisterBit(QMI8658_REG_CTRL3, 7) : clrRegisterBit(QMI8658_REG_CTRL3, 7);

        if (en) {
            enableGyroscope();
        }

        return DEV_WIRE_NONE;
    }


    int configFIFO(FIFO_Mode    mode,
                   FIFO_Samples samples = FIFO_SAMPLES_16,
                   IntPin pin = INTERRUPT_PIN_DISABLE,  //Disable interrupt mode
                   uint8_t trigger_samples = 16)
    {
        bool enGyro = isEnableGyroscope();
        bool enAccel = isEnableAccelerometer();

        if (enGyro) {
            disableGyroscope();
        }

        if (enAccel) {
            disableAccelerometer();
        }

        // Reset FIFO configure
        if (writeCommand(CTRL_CMD_RST_FIFO) != DEV_WIRE_NONE) {
            log_e("Reset fifo failed!");
            return DEV_WIRE_ERR;
        }

        __fifo_interrupt = true;

        switch (pin) {
        case INTERRUPT_PIN_1:
            setRegisterBit(QMI8658_REG_CTRL1, 2);
            break;
        case INTERRUPT_PIN_2:
            clrRegisterBit(QMI8658_REG_CTRL1, 2);
            break;
        case INTERRUPT_PIN_DISABLE:
            // Saves whether the fifo interrupt pin is enabled
            __fifo_interrupt  = false;
            break;
        default:
            break;
        }

        // Set fifo mode and samples len
        __fifo_mode = (samples << 2) | mode;
        if (writeRegister(QMI8658_REG_FIFO_CTRL, __fifo_mode) == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }

        /*
        * The FIFO_WTM register(0x13) indicates the expected level of FIFO data that host wants to get the FIFO Watermark interrupt.
        * The unit is sample, which means 6 bytes if one of accelerometer and gyroscope is enabled, and 12 bytes if both are enabled.
        * */
        if (writeRegister(QMI8658_REG_FIFO_WTM_TH, trigger_samples ) == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }

        if (enGyro) {
            enableGyroscope();
        }

        if (enAccel) {
            enableAccelerometer();
        }

        int res =  readRegister(QMI8658_REG_FIFO_CTRL);
        log_d("QMI8658_REG_FIFO_CTRL : 0x%X", res);
        if ((res & 0x02) == 0x02) {
            log_d("Enabled Stream mode.");
        } else if ((res & 0x01) == 0x01) {
            log_d("Enabled FIFO mode.");
        } else if ((res & 0x03) == 0x00) {
            log_d("Disabled FIFO.");
        }
        res >>= 2;
        if ((res & 0x03) == 0x03) {
            log_d("128 samples.");
        } else if ((res & 0x02) == 0x02) {
            log_d("64 samples.");
        } else if ((res & 0x01) == 0x01) {
            log_d("32 samples.");
        } else if ((res & 0x03) == 0x00) {
            log_d("16 samples.");
        }

        return DEV_WIRE_NONE;
    }

    uint16_t readFromFifo(IMUdata *acc, uint16_t accLength, IMUdata *gyro, uint16_t gyrLength)
    {
        if (__fifo_mode == FIFO_MODE_BYPASS) {
            log_e("FIFO is not configured.");
            return 0;
        }

        if (!__gyro_enabled && !__accel_enabled) {
            log_e("Sensor not enabled.");
            return 0;
        }

        uint16_t data_bytes = readFromFifo();
        if (data_bytes == 0) {
            return 0;
        }

        if (!__fifo_buffer) {
            log_e("FIFO buffer is NULL");
            return 0;
        }

        uint8_t enabled_sensor_count = (__accel_enabled && __gyro_enabled) ? 2 : 1;
        uint16_t samples_per_sensor = data_bytes / (6 * enabled_sensor_count);
        uint16_t total_samples = samples_per_sensor * enabled_sensor_count;

        log_d("Total samples: %u", total_samples);

        uint16_t accel_index = 0;
        uint16_t gyro_index = 0;

        for (uint16_t i = 0; i < total_samples; ++i) {
            auto data = reinterpret_cast<int16_t *>(&__fifo_buffer[i * 6]);
            int16_t x = data[0];
            int16_t y = data[1];
            int16_t z = data[2];

            if (__accel_enabled && __gyro_enabled) {
                if (i % 2 == 0) {
                    // Accel
                    if (accel_index < accLength) {
                        acc[accel_index].x = x * accelScales;
                        acc[accel_index].y = y * accelScales;
                        acc[accel_index].z = z * accelScales;
                        accel_index++;
                    }
                } else {
                    // Gyro
                    if (gyro_index < gyrLength) {
                        gyro[gyro_index].x = x * gyroScales;
                        gyro[gyro_index].y = y * gyroScales;
                        gyro[gyro_index].z = z * gyroScales;
                        gyro_index++;
                    }
                }
            } else if (__accel_enabled) {
                if (accel_index < accLength) {
                    acc[accel_index].x = x * accelScales;
                    acc[accel_index].y = y * accelScales;
                    acc[accel_index].z = z * accelScales;
                    accel_index++;
                }
            } else if (__gyro_enabled) {
                if (gyro_index < gyrLength) {
                    gyro[gyro_index].x = x * gyroScales;
                    gyro[gyro_index].y = y * gyroScales;
                    gyro[gyro_index].z = z * gyroScales;
                    gyro_index++;
                }
            }
        }
        return samples_per_sensor;
    }


private:

    uint16_t getFifoNeedBytes()
    {
        uint8_t sam[] = {16, 32, 64, 128};
        uint8_t sensors  = 0;
        if (__gyro_enabled && __accel_enabled) {
            sensors = 2;
        } else if (__gyro_enabled || __accel_enabled) {
            sensors = 1;
        }
        uint8_t samples =  ((__fifo_mode >> 2) & 0x03) ;
        return sam[samples] * 6 * sensors;
    }

    /**
     * @brief  readFromFifo
     * @note   Read the data in the FIFO buffer. configFIFO should be called before use.
     * @retval Returns the size of the element read
     */
    uint16_t readFromFifo()
    {
        uint8_t  status[2];
        uint16_t fifo_bytes   = 0;

        if ((__irq != -1) && __fifo_interrupt) {
            /*
             * Once the corresponds INT pin is configured to the push-pull mode, the FIFO watermark interrupt can be seen on the
             * corresponds INT pin. It will keep high level as long as the FIFO filled level is equal to or higher than the watermark, will
             * drop to low level as long as the FIFO filled level is lower than the configured FIFO watermark after reading out by host
             * and FIFO_RD_MODE is cleared.
            */
            if (this->getGpioLevel(__irq) == LOW) {
                return false;
            }
        }

        size_t alloc_size = getFifoNeedBytes();
        if (!__fifo_buffer) {
            __fifo_buffer = (uint8_t *)calloc(alloc_size, sizeof(uint8_t));
            if (!__fifo_buffer) {
                log_e("Calloc buffer size %u bytes failed!", alloc_size);
                return 0;
            }
            __fifo_size = alloc_size;

        } else if (alloc_size > __fifo_size) {
            __fifo_buffer = (uint8_t *)realloc(__fifo_buffer, alloc_size);
            if (!__fifo_buffer) {
                log_e("Realloc buffer size %u bytes failed!", alloc_size);
                return 0;
            }
        }

        // 1.Got FIFO watermark interrupt by INT pin or polling the FIFO_STATUS register (FIFO_WTM and/or FIFO_FULL).
        int val = readRegister(QMI8658_REG_FIFO_STATUS);
        if (val == DEV_WIRE_ERR) {
            return 0;
        }
        log_d("FIFO status:0x%x", val);

        if (!(val & _BV(4))) {
            log_d("FIFO is Empty");
            return 0;
        }
        if (val & _BV(5)) {
            log_d("FIFO Overflow condition has happened (data dropping happened)");
            // return 0;
        }
        if (val & _BV(6)) {
            log_d("FIFO Water Mark Level Hit");
        }
        if (val & _BV(7)) {
            log_d("FIFO is Full");
        }

        // 2.Read the FIFO_SMPL_CNT and FIFO_STATUS registers, to calculate the level of FIFO content data, refer to 8.4 FIFO Sample Count.
        if (readRegister(QMI8658_REG_FIFO_COUNT, status, 2) == DEV_WIRE_ERR) {
            log_e("Bus communication failed!");
            return 0;
        }

        // FIFO_Sample_Count (in byte) = 2 * (fifo_smpl_cnt_msb[1:0] * 256 + fifo_smpl_cnt_lsb[7:0])
        fifo_bytes = 2 * (((status[1] & 0x03)) << 8 | status[0]);

        log_d("reg fifo_bytes:%d ", fifo_bytes);

        //Samples 16  * 6 * 2  = 192
        //Samples 32  * 6 * 2  = 384
        //Samples 64  * 6 * 2  = 768
        //Samples 128 * 6 * 2  = 1536

        // 3.Send CTRL_CMD_REQ_FIFO (0x05) by CTRL9 command, to enable FIFO read mode. Refer to CTRL_CMD_REQ_FIFO for details.
        if (writeCommand(CTRL_CMD_REQ_FIFO) != DEV_WIRE_NONE) {
            log_e("Request FIFO failed!");
            return 0;
        }
        // 4.Read from the FIFO_DATA register per FIFO_Sample_Count.
        if (readRegister(QMI8658_REG_FIFO_DATA, __fifo_buffer, fifo_bytes) == DEV_WIRE_ERR) {
            log_e("Request FIFO data failed !");
            return 0;
        }

        // 5.Disable the FIFO Read Mode by setting FIFO_CTRL.FIFO_rd_mode to 0. New data will be filled into FIFO afterwards.
        if (writeRegister(QMI8658_REG_FIFO_CTRL, __fifo_mode) == DEV_WIRE_ERR) {
            log_e("Clear FIFO flag failed!");
            return 0;
        }

        return fifo_bytes;
    }

public:


    bool enableAccelerometer()
    {
        if (setRegisterBit(QMI8658_REG_CTRL7, 0)) {
            __accel_enabled = true;
        }
        return __accel_enabled;
    }

    bool disableAccelerometer()
    {
        if (clrRegisterBit(QMI8658_REG_CTRL7, 0)) {
            __accel_enabled = false;
            return true;
        }
        return false;
    }

    bool isEnableAccelerometer()
    {
        return __accel_enabled;
    }

    bool isEnableGyroscope()
    {
        return __gyro_enabled;
    }

    bool enableGyroscope()
    {
        if (setRegisterBit(QMI8658_REG_CTRL7, 1)) {
            __gyro_enabled = true;
        }
        return __gyro_enabled;
    }

    bool disableGyroscope()
    {
        if (clrRegisterBit(QMI8658_REG_CTRL7, 1)) {
            __gyro_enabled = false;
            return true;
        }
        return false;
    }

    bool getAccelRaw(int16_t *rawBuffer)
    {
        if (!__accel_enabled) {
            return false;
        }
        uint8_t buffer[6] = {0};
        if (readRegister(QMI8658_REG_AX_L, buffer, 6) != DEV_WIRE_ERR) {
            rawBuffer[0] = (int16_t)(buffer[1] << 8) | (buffer[0]);
            rawBuffer[1] = (int16_t)(buffer[3] << 8) | (buffer[2]);
            rawBuffer[2] = (int16_t)(buffer[5] << 8) | (buffer[4]);
        } else {
            return false;
        }
        return true;
    }

    bool getAccelerometer(float &x, float &y, float &z)
    {
        if (!__accel_enabled) {
            return false;
        }
        int16_t raw[3];
        if (getAccelRaw(raw)) {
            x = raw[0] * accelScales;
            y = raw[1] * accelScales;
            z = raw[2] * accelScales;
            return true;
        }
        return false;
    }

    float getAccelerometerScales()
    {
        return accelScales;
    }

    float getGyroscopeScales()
    {
        return gyroScales;
    }

    bool getGyroRaw(int16_t *rawBuffer)
    {
        if (!__gyro_enabled) {
            return false;
        }
        uint8_t buffer[6] = {0};
        if (readRegister(QMI8658_REG_GX_L, buffer, 6) != DEV_WIRE_ERR) {
            rawBuffer[0] = (int16_t)(buffer[1] << 8) | (buffer[0]);
            rawBuffer[1] = (int16_t)(buffer[3] << 8) | (buffer[2]);
            rawBuffer[2] = (int16_t)(buffer[5] << 8) | (buffer[4]);
        } else {
            return false;
        }
        return true;
    }

    int getGyroscope(float &x, float &y, float &z)
    {
        if (!__gyro_enabled) {
            return false;
        }
        int16_t raw[3];
        if (getGyroRaw(raw)) {
            x = raw[0] * gyroScales;
            y = raw[1] * gyroScales;
            z = raw[2] * gyroScales;
            return true;
        }
        return false;
    }

    bool getDataReady()
    {
        if ((__irq_enable_mask & 0x03) && (__irq != -1)) {
            if (this->getGpioLevel(__irq)) {
                return false;
            }
        }

        switch (sampleMode) {
        case SYNC_MODE:
            return  getRegisterBit(QMI8658_REG_STATUS_INT, 1);
        case ASYNC_MODE:
            //TODO: When Accel and Gyro are configured with different rates, this will always be false
            if (__accel_enabled & __gyro_enabled) {
                return readRegister(QMI8658_REG_STATUS0) & 0x03;
            } else if (__gyro_enabled) {
                return readRegister(QMI8658_REG_STATUS0) & 0x02;
            } else if (__accel_enabled) {
                return readRegister(QMI8658_REG_STATUS0) & 0x01;
            }
            break;
        default:
            break;
        }
        return false;
    }

    int enableSyncSampleMode()
    {
        sampleMode = SYNC_MODE;
        return setRegisterBit(QMI8658_REG_CTRL7, 7);
    }

    int disableSyncSampleMode()
    {
        sampleMode = ASYNC_MODE;
        return clrRegisterBit(QMI8658_REG_CTRL7, 7);
    }

    int enableLockingMechanism()
    {
        enableSyncSampleMode();
        if (writeRegister(QMI8658_REG_CAL1_L, 0x01) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        return writeCommand(CTRL_CMD_AHB_CLOCK_GATING);

    }

    int disableLockingMechanism()
    {
        disableSyncSampleMode();
        if (writeRegister(QMI8658_REG_CAL1_L, 0x00) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        return writeCommand(CTRL_CMD_AHB_CLOCK_GATING);
    }

    void dumpCtrlRegister()
    {
        uint8_t buffer[9];
        readRegister(QMI8658_REG_CTRL1, buffer, 9);
        for (int i = 0; i < 9; ++i) {
#if defined(ARDUINO)
            Serial.printf("CTRL%d: REG:0x%02X HEX:0x%02X\n", i + 1, QMI8658_REG_CTRL1 + i, buffer[i]);
#else
            printf("CTRL%d: 0x%02x\n", i + 1, buffer[i]);
#endif
        }

        buffer[0] =  readRegister(QMI8658_REG_FIFO_CTRL);
#if defined(ARDUINO)
        Serial.printf("FIFO_CTRL: REG:0x%02X HEX:0x%02X\n",  QMI8658_REG_FIFO_CTRL, buffer[0]);
#else
        printf("FIFO_CTRL: REG:0x%02X HEX:0x%02X\n",  QMI8658_REG_FIFO_CTRL, buffer[0]);
#endif

    }

    void powerDown()
    {
        disableAccelerometer();
        disableGyroscope();
        setRegisterBit(QMI8658_REG_CTRL1, 1);
    }

    void powerOn()
    {
        clrRegisterBit(QMI8658_REG_CTRL1, 1);
    }

    int getStatusRegister()
    {
        return readRegister(QMI8658_REG_STATUS1);
    }

    int configActivityInterruptMap(IntPin pin)
    {
        return pin == INTERRUPT_PIN_1 ? setRegisterBit(QMI8658_REG_CTRL8, 6)
               : clrRegisterBit(QMI8658_REG_CTRL8, 6);
    }

    /**
     * @brief configPedometer
     * @note The calculation of the Pedometer Detection is based on the accelerometer ODR defined by CTRL2.aODR,
     *      refer to Table 22 for details.
     * @param  ped_sample_cnt: Indicates the count of sample batch/window for calculation
     * @param  ped_fix_peak2peak:Indicates the threshold of the valid peak-to-peak detection
     *                          E.g., 0x00CC means 200mg
     * @param  ped_fix_peak:Indicates the threshold of the peak detection comparing to average
     *                      E.g., 0x0066 means 100mg
     * @param  ped_time_up:Indicates the maximum duration (timeout window) for a step.
     *                     Reset counting calculation if no peaks detected within this duration.
     *                    E.g., 80 means 1.6s @ ODR = 50Hz
     * @param  ped_time_low:Indicates the minimum duration for a step.
     *                     The peaks detected within this duration (quiet time) is ignored.
     *                    E.g., 12 means 0.25s @ ODR = 50Hz
     * @param  ped_time_cnt_entry:Indicates the minimum continuous steps to start the valid step counting.
     *                           If the continuously detected steps is lower than this count and timeout,
     *                           the steps will not be take into account;
     *                           if yes, the detected steps will all be taken into account and
     *                           counting is started to count every following step before timeout.
     *                           This is useful to screen out the fake steps detected by non-step vibrations
     *                           The timeout duration is defined by ped_time_up.
     *                           E.g., 10 means 10 steps entry count
     * @param  ped_fix_precision:0 is recommended
     * @param  ped_sig_count: The amount of steps when to update the pedometer output registers.
     *                      E.g., ped_sig_count = 4, every 4 valid steps is detected, update the registers once (added by 4).
     * @retval
     */
    int configPedometer(uint16_t ped_sample_cnt, uint16_t ped_fix_peak2peak, uint16_t ped_fix_peak,
                        uint16_t ped_time_up, uint8_t ped_time_low = 0x14, uint8_t ped_time_cnt_entry = 0x0A, uint8_t ped_fix_precision = 0x00,
                        uint8_t ped_sig_count = 0x04)
    {
        // The Pedometer can only work in Non-SyncSample mode
        disableSyncSampleMode();

        bool enGyro = isEnableGyroscope();
        bool enAccel = isEnableAccelerometer();

        if (enGyro) {
            disableGyroscope();
        }

        if (enAccel) {
            disableAccelerometer();
        }

        writeRegister(QMI8658_REG_CAL1_L, ped_sample_cnt & 0xFF);
        writeRegister(QMI8658_REG_CAL1_H, (ped_sample_cnt >> 8) & 0xFF);
        writeRegister(QMI8658_REG_CAL2_L, ped_fix_peak2peak & 0xFF);
        writeRegister(QMI8658_REG_CAL2_H, (ped_fix_peak2peak >> 8) & 0xFF);
        writeRegister(QMI8658_REG_CAL3_L, ped_fix_peak & 0xFF);
        writeRegister(QMI8658_REG_CAL3_H, (ped_fix_peak >> 8) & 0xFF);
        writeRegister(QMI8658_REG_CAL4_H, 0x01);
        writeRegister(QMI8658_REG_CAL4_L, 0x02);

        writeCommand(CTRL_CMD_CONFIGURE_PEDOMETER);

        writeRegister(QMI8658_REG_CAL1_L, ped_time_up & 0xFF);
        writeRegister(QMI8658_REG_CAL1_H, (ped_time_up >> 8) & 0xFF);
        writeRegister(QMI8658_REG_CAL2_L, ped_time_low);
        writeRegister(QMI8658_REG_CAL2_H, ped_time_cnt_entry);
        writeRegister(QMI8658_REG_CAL3_L, ped_fix_precision);
        writeRegister(QMI8658_REG_CAL3_H, ped_sig_count);
        writeRegister(QMI8658_REG_CAL4_H, 0x02);
        writeRegister(QMI8658_REG_CAL4_L, 0x02);

        writeCommand(CTRL_CMD_CONFIGURE_PEDOMETER);

        if (enGyro) {
            enableGyroscope();
        }

        if (enAccel) {
            enableAccelerometer();
        }
        return 0;
    }

    uint32_t getPedometerCounter()
    {
        uint8_t buffer[3];
        if (readRegister(QMI8658_REG_STEP_CNT_LOW, buffer, 3) != DEV_WIRE_ERR) {
            return (uint32_t)(((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0]);
        }
        return 0;
    }

    int clearPedometerCounter()
    {
        return writeCommand(CTRL_CMD_RESET_PEDOMETER);
    }

    // The Pedometer can only work in Non-SyncSample mode
    bool enablePedometer(IntPin pin = INTERRUPT_PIN_DISABLE)
    {
        if (!__accel_enabled)return false;

        switch (pin) {
        case INTERRUPT_PIN_1:
        case INTERRUPT_PIN_2:
            configActivityInterruptMap(pin);
            enableINT(pin);
            break;
        default:
            break;
        }
        return setRegisterBit(QMI8658_REG_CTRL8, 4);
    }

    bool disablePedometer()
    {
        if (!__accel_enabled)return false;
        return clrRegisterBit(QMI8658_REG_CTRL8, 4);
    }



    /**
     * @brief   configTap
     * @note    The calculation of the Tap Detection is based on the accelerometer ODR defined by CTRL2.aODR, refer to Table 22 for details.
     * @param  priority: Priority definition between the x, y, z axes of acceleration. Only
                         Priority[2:0] bits are used.
                         The axis that output the first peak of Linear Acceleration in a valid
                         Tap detection, will be consider as the Tap axis. However, there is
                         possibility that two or three of the axes shows same Linear
                         Acceleration at exactly same time when reach (or be higher than)
                         the PeakMagThr. In this case, the defined priority is used to judge
                         and select the axis as Tap axis.
     * @param  peakWindow:Defines the maximum duration (in sample) for a valid peak. In a
                        valid peak, the linear acceleration should reach or be higher than
                        the PeakMagThr and should return to quiet (no significant
                        movement) within UDMThr, at the end of PeakWindow. E.g., 20 @500Hz ODR
     * @param  tapWindow:Defines the minimum quiet time before the second Tap happen.
                        After the first Tap is detected, there should be no significant
                        movement (defined by UDMThr) during the TapWindow. The valid
                        second tap should be detected after TapWindow and before
                        DTapWindow. E.g., 50 @500Hz ODR
     * @param  dTapWindow:Defines the maximum time for a valid second Tap for Double Tap,
                        count start from the first peak of the valid first Tap.  E.g., 250 @500Hz ODR
     * @param  alpha:Defines the ratio for calculation the average of the acceleration.
                    The bigger of Alpha, the bigger weight of the latest data.  E.g., 0.0625
     * @param  gamma:Defines the ratio for calculating the average of the movement
                    magnitude. The bigger of Gamma, the bigger weight of the latest data. E.g., 0.25
     * @param  peakMagThr:Threshold for peak detection.  E.g, 0.8g2
     * @param  UDMThr:Undefined Motion threshold. This defines the threshold of the
                    Linear Acceleration for quiet status. E.g., 0.4g2
     * @retval
     */
    int configTap(uint8_t priority, uint8_t peakWindow, uint16_t tapWindow, uint16_t dTapWindow,
                  float alpha, float gamma, float peakMagThr, float UDMThr)
    {

        // The Tap detection can only work in Non-SyncSample mode
        disableSyncSampleMode();

        bool enGyro = isEnableGyroscope();
        bool enAccel = isEnableAccelerometer();

        if (enGyro) {
            disableGyroscope();
        }

        if (enAccel) {
            disableAccelerometer();
        }
        writeRegister(QMI8658_REG_CAL1_L, peakWindow);
        writeRegister(QMI8658_REG_CAL1_H, priority);
        writeRegister(QMI8658_REG_CAL2_L, tapWindow & 0xFF);
        writeRegister(QMI8658_REG_CAL2_H, (tapWindow >> 8) & 0xFF);
        writeRegister(QMI8658_REG_CAL3_L, dTapWindow & 0xFF);
        writeRegister(QMI8658_REG_CAL3_H, (dTapWindow >> 8) & 0xFF);
        // writeRegister(QMI8658_REG_CAL4_L, 0x02);
        writeRegister(QMI8658_REG_CAL4_H, 0x01);

        writeCommand(CTRL_CMD_CONFIGURE_TAP);

        // 1-byte unsigned,7-bits fraction
        uint8_t alphaHex = (uint8_t)(alpha * 128);
        writeRegister(QMI8658_REG_CAL1_L, alphaHex);

        // 1-byte unsigned,7-bits fraction
        uint8_t gammaHex = (uint8_t)(gamma * 128);
        writeRegister(QMI8658_REG_CAL1_H, gammaHex);

        const double g = 9.81; // Earth's gravitational acceleration m/s^2
        double resolution = 0.001 * g * g; // Calculation resolution  0.001g^2

        double acceleration_square = peakMagThr * g * g;     // Calculate the square of the acceleration
        uint16_t value = (uint16_t)(acceleration_square / resolution); // Calculates the value of a 2-byte unsigned integer

        writeRegister(QMI8658_REG_CAL2_L, lowByte(value));
        writeRegister(QMI8658_REG_CAL2_H, highByte(value));

        acceleration_square = UDMThr * g * g;     // Calculate the square of the acceleration
        value = (uint16_t)(acceleration_square / resolution); // Calculates the value of a 2-byte unsigned integer

        writeRegister(QMI8658_REG_CAL3_L, lowByte(value));
        writeRegister(QMI8658_REG_CAL3_H, highByte(value));
        // writeRegister(QMI8658_REG_CAL4_L, 0x02);
        writeRegister(QMI8658_REG_CAL4_H, 0x02);

        writeCommand(CTRL_CMD_CONFIGURE_TAP);

        if (enGyro) {
            enableGyroscope();
        }

        if (enAccel) {
            enableAccelerometer();
        }

        return 0;
    }

    bool enableTap(IntPin pin = INTERRUPT_PIN_DISABLE)
    {
        if (!__accel_enabled)return false;
        switch (pin) {
        case INTERRUPT_PIN_1:
        case INTERRUPT_PIN_2:
            configActivityInterruptMap(pin);
            enableINT(pin);
            break;
        default:
            break;
        }
        return setRegisterBit(QMI8658_REG_CTRL8, 0);
    }

    bool disableTap()
    {
        return clrRegisterBit(QMI8658_REG_CTRL8, 0);
    }

    TapEvent getTapStatus()
    {
        int val = readRegister(QMI8658_REG_TAP_STATUS);
        if (val & _BV(7)) {
            log_i("Tap was detected on the negative direction of the Tap axis");
        } else {
            log_i("Tap was detected on the positive direction of the Tap axis");
        }
        uint8_t t = (val >> 4) & 0x03;
        switch (t) {
        case 0:
            log_i("No Tap was detected");
            break;
        case 1:
            log_i("Tap was detected on X axis");
            break;
        case 2:
            log_i("Tap was detected on Y axis");
            break;
        case 3:
            log_i("Tap was detected on Z axis");
            break;
        default:
            break;
        }
        t = val & 0x03;
        switch (t) {
        case 0:
            log_i("No Tap was detected");
            return INVALID_TAP;
        case 1:
            log_i("Single-Tap was detected");
            return SINGLE_TAP;
        case 2:
            log_i("Double-Tap was detected");
            return DOUBLE_TAP;
        default:
            break;
        }
        return INVALID_TAP;
    }


    //TODO:Need Test
    int configMotion(
        //* See enum MotionCtrl
        uint8_t modeCtrl,
        //* Define the slope threshold of the x-axis for arbitrary motion detection
        float AnyMotionXThr,
        //* Define the slope threshold of the y-axis for arbitrary motion detection
        float AnyMotionYThr,
        //* Define the slope threshold of the z-axis for arbitrary motion detection
        float AnyMotionZThr,
        //* Defines the minimum number of consecutive samples (duration) that the absolute
        //* of the slope of the enabled axis/axes data should keep higher than the threshold
        uint8_t AnyMotionWindow,
        //* Defines the slope threshold of the x-axis for no motion detection
        float NoMotionXThr,
        //* Defines the slope threshold of the y-axis for no motion detection
        float NoMotionYThr,
        //* Defines the slope threshold of the z-axis for no motion detection
        float NoMotionZThr,
        //* Defines the minimum number of consecutive samples (duration) that the absolute
        //* of the slope of the enabled axis/axes data should keep lower than the threshold
        uint8_t NoMotionWindow,
        //* Defines the wait window (idle time) starts from the first Any-Motion event until
        //* starting to detecting another Any-Motion event form confirmation
        uint16_t SigMotionWaitWindow,
        //* Defines the maximum duration for detecting the other Any-Motion
        //* event to confirm Significant-Motion, starts from the first Any -Motion event
        uint16_t SigMotionConfirmWindow)
    {
        // Only work in Non-SyncSample mode
        disableSyncSampleMode();

        bool enGyro = isEnableGyroscope();
        bool enAccel = isEnableAccelerometer();

        if (enGyro) {
            disableGyroscope();
        }

        if (enAccel) {
            disableAccelerometer();
        }

        writeRegister(QMI8658_REG_CAL1_L, mgToBytes(AnyMotionXThr));
        writeRegister(QMI8658_REG_CAL1_H, mgToBytes(AnyMotionYThr));
        writeRegister(QMI8658_REG_CAL2_L, mgToBytes(AnyMotionZThr));
        writeRegister(QMI8658_REG_CAL2_H, mgToBytes(NoMotionXThr));
        writeRegister(QMI8658_REG_CAL3_L, mgToBytes(NoMotionYThr));
        writeRegister(QMI8658_REG_CAL3_H, mgToBytes(NoMotionZThr));
        writeRegister(QMI8658_REG_CAL4_L, modeCtrl);
        writeRegister(QMI8658_REG_CAL4_H, 0x01);

        writeCommand(CTRL_CMD_CONFIGURE_MOTION);

        writeRegister(QMI8658_REG_CAL1_L, AnyMotionWindow);
        writeRegister(QMI8658_REG_CAL1_H, NoMotionWindow);
        writeRegister(QMI8658_REG_CAL2_L, lowByte(SigMotionWaitWindow));
        writeRegister(QMI8658_REG_CAL2_H, highByte(SigMotionWaitWindow));
        writeRegister(QMI8658_REG_CAL3_L, lowByte(SigMotionConfirmWindow));
        writeRegister(QMI8658_REG_CAL3_H, highByte(SigMotionConfirmWindow));
        // writeRegister(QMI8658_REG_CAL4_L, 0x02);
        writeRegister(QMI8658_REG_CAL4_H, 0x02);

        writeCommand(CTRL_CMD_CONFIGURE_MOTION);

        if (enGyro) {
            enableGyroscope();
        }

        if (enAccel) {
            enableAccelerometer();
        }
        return 0;
    }

    bool enableMotionDetect(IntPin pin = INTERRUPT_PIN_DISABLE)
    {
        if (!__accel_enabled)return false;
        switch (pin) {
        case INTERRUPT_PIN_1:
        case INTERRUPT_PIN_2:
            configActivityInterruptMap(pin);
            enableINT(pin);
            break;
        default:
            break;
        }
        setRegisterBit(QMI8658_REG_CTRL8, 1);
        setRegisterBit(QMI8658_REG_CTRL8, 2);
        setRegisterBit(QMI8658_REG_CTRL8, 3);
        return true;
    }

    bool disableMotionDetect()
    {
        clrRegisterBit(QMI8658_REG_CTRL8, 1);
        clrRegisterBit(QMI8658_REG_CTRL8, 2);
        clrRegisterBit(QMI8658_REG_CTRL8, 3);
        return false;
    }


    /**
     * @brief  configWakeOnMotion
     * @note   Configuring Wom will reset the sensor, set the function to Wom, and there will be no data output
     * @param  WoMThreshold: Resolution = 1mg ,default 200mg
     * @param  odr: Accelerometer output data rate  ,default low power 128Hz
     * @param  pin: Interrupt Pin( 1 or 2 ) ,default use pin2
     * @param  defaultPinValue: WoM Interrupt Initial Value select: ,default pin high
     *  01 – INT2 (with initial value 0)
     *  11 – INT2 (with initial value 1)
     *  00 – INT1 (with initial value 0)
     *  10 – INT1 (with initial value 1)
     * @param  blankingTime: Interrupt Blanking Time
     *  (in number of accelerometer samples), the
     *  number of consecutive samples that will be ignored after
     *  enabling the WoM, to screen out unwanted fake detection
     * @retval
     */
    int configWakeOnMotion(uint8_t WoMThreshold = 200,
                           AccelODR odr = ACC_ODR_LOWPOWER_128Hz,
                           IntPin pin = INTERRUPT_PIN_2,
                           uint8_t defaultPinValue = 1,
                           uint8_t blankingTime = 0x20
                          )
    {

        uint8_t val = 0;

        //Reset default value
        if (!reset()) {
            return DEV_WIRE_ERR;
        }

        // Disable sensors
        clrRegisterBit(QMI8658_REG_CTRL7, 0);

        //setAccelRange
        if (writeRegister(QMI8658_REG_CTRL2, 0x8F, (ACC_RANGE_8G << 4)) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        // setAccelOutputDataRate
        if (writeRegister(QMI8658_REG_CTRL2, 0xF0, odr) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        //set wom
        if (writeRegister(QMI8658_REG_CAL1_L, WoMThreshold) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        if ( pin == INTERRUPT_PIN_1) {
            val = defaultPinValue ? 0x02 : 0x00;
        } else if (pin == INTERRUPT_PIN_2) {
            val = defaultPinValue ? 0x03 : 0x01;
        }

        val <<= 6;
        val |= (blankingTime & 0x3F);
        if (writeRegister(QMI8658_REG_CAL1_H, val) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        if (writeCommand(CTRL_CMD_WRITE_WOM_SETTING) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }

        enableAccelerometer();

        enableINT(pin);

        return DEV_WIRE_NONE;
    }



    void getChipUsid(uint8_t *buffer, uint8_t length)
    {
        if (length > 6) {
            length = 6;
        }
        memcpy(buffer, usid, length);
    }


    uint32_t getChipFirmwareVersion()
    {
        return revisionID;
    }

    /**
     * @brief update
     * @note  Get the interrupt status and status 0, status 1 of the sensor
     * @retval  Return SensorStatus
     */
    uint16_t update()
    {
        uint16_t result = 0;
        // STATUSINT 0x2D
        // STATUS0 0x2E
        // STATUS1 0x2F
        uint8_t status[3];
        if (readRegister(QMI8658_REG_STATUS_INT, status, 3) != DEV_WIRE_NONE) {
            return 0;
        }

        // log_i("STATUSINT:0x%X BIN:", status[0]);
        // log_i("STATUS0:0x%X BIN:", status[1]);
        // log_i("STATUS1:0x%X BIN:", status[2]);
        // log_i("------------------\n");

        // Ctrl9 CmdDone
        // Indicates CTRL9 Command was done, as part of CTRL9 protocol
        // 0: Not Completed
        // 1: Done
        if (status[0] & 0x80) {
            result |= STATUS_INT_CTRL9_CMD_DONE;
        }
        // If syncSample (CTRL7.bit7) = 1:
        //      0: Sensor Data is not locked.
        //      1: Sensor Data is locked.
        // If syncSample = 0, this bit shows the same value of INT1 level
        if (status[0] & 0x02) {
            result |= STATUS_INT_LOCKED;
        }
        // If syncSample (CTRL7.bit7) = 1:
        //      0: Sensor Data is not available
        //      1: Sensor Data is available for reading
        // If syncSample = 0, this bit shows the same value of INT2 level
        if (status[0] & 0x01) {
            result |= STATUS_INT_AVAIL;
            // if (eventGyroDataReady)eventGyroDataReady();
            // if (eventAccelDataReady)eventAccelDataReady();
        }

        //Locking Mechanism Can reading..
        if ((status[0] & 0x03) == 0x03) {
            if (eventDataLocking)eventDataLocking();
        }

        //=======================================
        // Valid only in asynchronous mode
        if (sampleMode == ASYNC_MODE) {
            // Gyroscope new data available
            // 0: No updates since last read.
            // 1: New data available
            if (status[1] & 0x02) {
                result |= STATUS0_GYRO_DATA_READY;
                if (eventGyroDataReady)eventGyroDataReady();
                __gDataReady = true;
            }
            // Accelerometer new data available
            // 0: No updates since last read.
            // 1: New data available.
            if (status[1] & 0x01) {
                result |= STATUS0_ACCEL_DATA_READY;
                if (eventAccelDataReady)eventAccelDataReady();
                __aDataReady = true;
            }
        }

        //=======================================
        // Significant Motion
        // 0: No Significant-Motion was detected
        // 1: Significant-Motion was detected
        if (status[2] & 0x80) {
            result |= STATUS1_SIGNIFICANT_MOTION;
            if (eventSignificantMotion)eventSignificantMotion();
        }
        // No Motion
        // 0: No No-Motion was detected
        // 1: No-Motion was detected
        if (status[2] & 0x40) {
            result |= STATUS1_NO_MOTION;
            if (eventNoMotionEvent)eventNoMotionEvent();
        }
        // Any Motion
        // 0: No Any-Motion was detected
        // 1: Any-Motion was detected
        if (status[2] & 0x20) {
            result |= STATUS1_ANY_MOTION;
            if (eventAnyMotionEvent)eventAnyMotionEvent();
        }
        // Pedometer
        // 0: No step was detected
        // 1: step was detected
        if (status[2] & 0x10) {
            result |= STATUS1_PEDOMETER_MOTION;
            if (eventPedometerEvent)eventPedometerEvent();
        }
        // WoM
        // 0: No WoM was detected
        // 1: WoM was detected
        if (status[2] & 0x04) {
            result |= STATUS1_WOM_MOTION;
            if (eventWomEvent)eventWomEvent();
        }
        // TAP
        // 0: No Tap was detected
        // 1: Tap was detected
        if (status[2] & 0x02) {
            result |= STATUS1_TAP_MOTION;
            if (eventTagEvent)eventTagEvent();
        }
        return result;
    }

    void setWakeupMotionEventCallBack(EventCallBack_t cb)
    {
        eventWomEvent = cb;
    }

    void setTapEventCallBack(EventCallBack_t cb)
    {
        eventTagEvent = cb;
    }

    void setPedometerEventCallBack(EventCallBack_t cb)
    {
        eventPedometerEvent = cb;
    }

    void setNoMotionEventCallBack(EventCallBack_t cb)
    {
        eventNoMotionEvent = cb;
    }

    void setAnyMotionEventCallBack(EventCallBack_t cb)
    {
        eventAnyMotionEvent = cb;
    }

    void setSignificantMotionEventCallBack(EventCallBack_t cb)
    {
        eventSignificantMotion = cb;
    }

    void setGyroDataReadyCallBack(EventCallBack_t cb)
    {
        eventGyroDataReady = cb;
    }

    void setAccelDataReadyEventCallBack(EventCallBack_t cb)
    {
        eventAccelDataReady = cb;
    }

    void setDataLockingEventCallBack(EventCallBack_t cb)
    {
        eventDataLocking = cb;
    }


    bool calibration(uint16_t *gX_gain = NULL, uint16_t *gY_gain = NULL, uint16_t *gZ_gain = NULL)
    {
        // 1.Set CTRL7.aEN = 0 and CTRL7.gEN = 0, to disable the accelerometer and gyroscope.
        if (writeRegister(QMI8658_REG_CTRL7, 0x00) != DEV_WIRE_NONE) {
            return false;
        }

        // 2.Issue the CTRL_CMD_ON_DEMAND_CALIBRATION (0xA2) by CTRL9 command.
        if (writeCommand(CTRL_CMD_ON_DEMAND_CALIBRATION, 3000) != DEV_WIRE_NONE) {
            return false;
        }

        // 3.And wait about 1.5 seconds for QMI8658A to finish the CTRL9 command.
        delay(1600);

        // 4.Read the COD_STATUS register (0x46) to check the result/status of the COD implementation.
        int result =  readRegister(QMI8658_REG_COD_STATUS);

        if (result == DEV_WIRE_ERR)return false;

        // During the process, it is recommended to place the device in quiet, otherwise, the COD might fail and report error.

        if (result & _BV(7)) {
            log_e("COD failed for checking low sensitivity limit of X axis of gyroscope");
            return false;
        }
        if (result & _BV(6)) {
            log_e("COD failed for checking high sensitivity limit of X axis of gyroscope");
            return false;
        }
        if (result & _BV(5)) {
            log_e("COD failed for checking low sensitivity limit of Y axis of gyroscope");
            return false;
        }
        if (result & _BV(4)) {
            log_e("COD failed for checking high sensitivity limit of Y axis of gyroscope");
            return false;
        }
        if (result & _BV(3)) {
            log_e("Accelerometer checked failed (significant vibration happened during COD)");
            return false;
        }
        if (result & _BV(2)) {
            log_e("Gyroscope startup failure happened when COD was called");
            return false;
        }
        if (result & _BV(1)) {
            log_e("COD was called while gyroscope was enabled, COD return failure");
            return false;
        }
        if (result & _BV(0)) {
            log_e("COD failed; no COD correction applied");
            return false;
        }
        log_d("All calibrations are completed");

        if (gX_gain && gY_gain && gZ_gain) {
            uint8_t rawBuffer[6] = {0};
            if (readRegister(QMI8658_REG_DVX_L, rawBuffer, 6) != DEV_WIRE_NONE) {
                return false;
            }
            *gX_gain = ((uint16_t)rawBuffer[0]) | (uint16_t)(rawBuffer[1] << 8);
            *gY_gain = ((uint16_t)rawBuffer[2]) | (uint16_t)(rawBuffer[3] << 8);
            *gZ_gain = ((uint16_t)rawBuffer[4]) | (uint16_t)(rawBuffer[5] << 8);
        }

        return true;
    }


    bool writeCalibration(uint16_t gX_gain, uint16_t gY_gain, uint16_t gZ_gain)
    {
        // 1. Disable Accelerometer and Gyroscope by setting CTRL7.aEN = 0 and CTRL7.gEN = 0
        if (writeRegister(QMI8658_REG_CTRL7, 0x00) != DEV_WIRE_NONE) {
            return false;
        }

        uint8_t buffer[] = {

            // 2. write Gyro-X gain (16 bits) to registers CAL1_L and CAL1_H registers (0x0B, 0x0C)
            lowByte(gX_gain),
            highByte(gX_gain),
            // 3. write Gyro-Y gain (16 bits) to registers CAL2_L and CAL2_H registers (0x0D, 0x0E)
            lowByte(gY_gain),
            highByte(gY_gain),
            // 4. write Gyro-Z gain (16 bits) to registers CAL3_L and CAL3_H registers (0x0F, 0x10)
            lowByte(gZ_gain),
            highByte(gZ_gain),
        };

        writeRegister(QMI8658_REG_CAL1_L, buffer, sizeof(buffer));

        // 5. Write 0xAA to CTRL9 and follow CTRL9 protocol
        if (writeCommand(CTRL_CMD_APPLY_GYRO_GAINS, 3000) != DEV_WIRE_NONE) {
            return false;
        }

        return true;
    }


    bool selfTestAccel()
    {
        // 1- Disable the sensors (CTRL7 = 0x00).
        if (writeRegister(QMI8658_REG_CTRL7, 0x00) != DEV_WIRE_NONE) {
            return false;
        }

        // 2- Set proper accelerometer ODR (CTRL2.aODR) and bit CTRL2.aST (bit7) to 1 to trigger the Self-Test.
        if (writeRegister(QMI8658_REG_CTRL2, 0xF0, ACC_ODR_1000Hz | 0x80) != DEV_WIRE_NONE) {
            return false;
        }

        // 3- Wait for QMI8658A to drive INT2 to High, if INT2 is enabled (CTRL1.bit4 = 1), or STATUSINT.bit0 is set to 1.
        int retry = 50;
        int dataReady = 0x00;
        while (dataReady != 0x01) {
            uint8_t reg_var = readRegister(QMI8658_REG_STATUS_INT);
            log_d("reg_var : %x", reg_var);
            dataReady = reg_var & 0x01;
            // dataReady = readRegister(QMI8658_REG_STATUS_INT) & 0x01;
            delay(20);
            if (--retry <= 0) {
                log_e("No response.");
                return false;
            }
        }

        log_i("Data is ready for reading....");

        //4- Set CTRL2.aST(bit7) to 0, to clear STATUSINT1.bit0 and/or INT2.
        clrRegisterBit(QMI8658_REG_CTRL2, 7);

        // 5- Check for QMI8658A drives INT2 back to Low, and sets STATUSINT1.bit0 to 0.
        retry = 50;
        while (dataReady == 0x01) {
            uint8_t reg_var = readRegister(QMI8658_REG_STATUS_INT);
            log_d("reg_var : %x", reg_var);
            dataReady = (reg_var & 0x01);
            // dataReady = !(readRegister(QMI8658_REG_STATUS_INT) & 0x01);
            delay(20);
            if (--retry <= 0) {
                log_e("No response.");
                return false;
            }
        }

        /*
            6- Read the Accel Self-Test result:
                X channel: dVX_L and dVX_H (registers 0x51 and 0x52)
                Y channel: dVY_L and dVY_H (registers 0x53 and 0x54)
                Z channel: dVZ_L and dVZ_H (registers 0x55 and 0x56)
                The results are 16-bits in format signed U5.11, resolution 0.5mg (1 / 2^11 g).
        */
        uint8_t rawBuffer[6];

        if (readRegister(QMI8658_REG_DVX_L, rawBuffer, 6) != DEV_WIRE_NONE) {
            return false;
        }

        int16_t dVX = (int16_t)(rawBuffer[0]) | (int16_t)(((int16_t)rawBuffer[1]) << 8);
        int16_t dVY = (int16_t)(rawBuffer[2]) | (int16_t)(((int16_t)rawBuffer[3]) << 8);
        int16_t dVZ = (int16_t)(rawBuffer[4]) | (int16_t)(((int16_t)rawBuffer[5]) << 8);

        // To convert to mg, considering the U5.11 format, we need to divide by (2^11) to get the actual mg value
        float dVX_mg = dVX * 0.5;   // 0.5mg is the smallest unit of this format
        float dVY_mg = dVY * 0.5;
        float dVZ_mg = dVZ * 0.5;

        log_d("\n\tdVX_mg:%05.11f \n\tdVY_mg:%05.11f \n\tdVZ_mg:%05.11f", dVX_mg, dVY_mg, dVZ_mg);
        // If the absolute results of all three axes are higher than 200mg, the accelerometer can be considered functional.
        // Otherwise, the accelerometer cannot be considered functional.
        if (abs(dVX_mg) > 200 && abs(dVY_mg) > 200 && abs(dVZ_mg) > 200) {
            Serial.println("Accelerometer is working properly.");
        } else {
            Serial.println("Accelerometer is not working properly.");
            return false;
        }
        return true;
    }


    bool selfTestGyro()
    {
        // 1- Disable the sensors (CTRL7 = 0x00).
        if (writeRegister(QMI8658_REG_CTRL7, 0x00) != DEV_WIRE_NONE) {
            return false;
        }

        // 2- Set the bit gST to 1. (CTRL3.bit7 = 1’b1).
        setRegisterBit(QMI8658_REG_CTRL3, 7);

        // 3- Wait for QMI8658A to drive INT2 to High, if INT2 is enabled, or STATUS_INT.bit0 is set to 1.
        int retry = 50;
        int dataReady = 0x00;
        while (dataReady != 0x01) {
            dataReady = readRegister(QMI8658_REG_STATUS_INT) & 0x01;
            delay(20);
            if (--retry <= 0) {
                log_e("No response.");
                return false;
            }
        }

        log_i("Data is ready for reading....");

        //4- Set CTRL3.aST(bit7) to 0, to clear STATUS_INT1.bit0 and/or INT2.
        clrRegisterBit(QMI8658_REG_CTRL3, 7);

        // 5- Check for QMI8658A drives INT2 back to Low, or sets STATUSINT1.bit0 to 0.
        retry = 50;
        while (dataReady != 0x00) {
            dataReady = !(readRegister(QMI8658_REG_STATUS_INT) & 0x01);
            delay(20);
            if (--retry <= 0) {
                log_e("No response.");
                return false;
            }
        }

        /*
            6- Read the Gyro Self-Test result:
                X channel: dVX_L and dVX_H (registers 0x51 and 0x52)
                Y channel: dVY_L and dVY_H (registers 0x53 and 0x54)
                Z channel: dVZ_L and dVZ_H (registers 0x55 and 0x56)
                Read the 16 bits result in format signed U12.4, resolution is 62.5mdps (1 / 2^4 dps).
        */
        uint8_t rawBuffer[6];
        if (readRegister(QMI8658_REG_DVX_L, rawBuffer, 6) != DEV_WIRE_NONE) {
            return false;
        }

        // int16_t x = (int16_t)(rawBuffer[0]) | ((int16_t)(rawBuffer[1] << 8));
        // int16_t y = (int16_t)(rawBuffer[2]) | ((int16_t)(rawBuffer[3] << 8));
        // int16_t z = (int16_t)(rawBuffer[4]) | ((int16_t)(rawBuffer[5] << 8));

        float dVX = (((int16_t)rawBuffer[0]) << 12) | ((int16_t)(rawBuffer[1]) >> 4);
        float dVY = (((int16_t)rawBuffer[2]) << 12) | ((int16_t)(rawBuffer[3]) >> 4);
        float dVZ = (((int16_t)rawBuffer[4]) << 12) | ((int16_t)(rawBuffer[5]) >> 4);

        dVX *= (1.0 / (1 << 4)); // 62.5 mdps
        dVY *= (1.0 / (1 << 4)); // 62.5 mdps
        dVZ *= (1.0 / (1 << 4)); // 62.5 mdps

        log_d("\n\tdVX:%12.4f \n\tdVY:%12.4f \n\tdVZ:%12.4f", dVX, dVY, dVZ);

        //  If the absolute results of all three axes are higher than 300dps, the gyroscope can be considered functional.
        // Otherwise, the gyroscope cannot be considered functional.
        if (abs(dVX) > 300 && abs(dVY) > 300 && abs(dVZ) > 300) {
            Serial.println("Gyro is working properly.");
        } else {
            Serial.println("Gyro is not working properly.");
            return false;
        }

        return true;
    }

    // This offset change is lost when the sensor is power cycled, or the system is reset
    // Each delta offset value should contain 16 bits and the format is signed 11.5 (5 fraction bits, unit is 1 / 2^5).
    void setAccelOffset(int16_t offset_x, int16_t offset_y, int16_t offset_z)
    {

        uint8_t data[6];
        data[0] = lowByte(offset_x);
        data[1] = highByte(offset_x);
        data[2] = lowByte(offset_y);
        data[3] = highByte(offset_y);
        data[4] = lowByte(offset_z);
        data[5] = highByte(offset_z);

        writeRegister(QMI8658_REG_CAL1_L, data, 2);
        writeRegister(QMI8658_REG_CAL2_L, data + 2, 2);
        writeRegister(QMI8658_REG_CAL3_L, data + 4, 2);

        writeCommand(CTRL_CMD_ACCEL_HOST_DELTA_OFFSET);
    }

    // This offset change is lost when the sensor is power cycled, or the system is reset
    // Each delta offset value should contain 16 bits and the format is signed 11.5 (5 fraction bits, unit is 1 / 2^5).
    void setGyroOffset(int16_t offset_x, int16_t offset_y, int16_t offset_z)
    {

        uint8_t data[6];
        data[0] = lowByte(offset_x);
        data[1] = highByte(offset_x);
        data[2] = lowByte(offset_y);
        data[3] = highByte(offset_y);
        data[4] = lowByte(offset_z);
        data[5] = highByte(offset_z);

        writeRegister(QMI8658_REG_CAL1_L, data, 2);
        writeRegister(QMI8658_REG_CAL2_L, data + 2, 2);
        writeRegister(QMI8658_REG_CAL3_L, data + 4, 2);

        writeCommand(CTRL_CMD_GYRO_HOST_DELTA_OFFSET);

    }

    void setPins(int irq)
    {
        __irq = irq;
    }

private:
    float accelScales, gyroScales;
    uint32_t lastTimestamp = 0;
    uint8_t sampleMode = ASYNC_MODE;
    bool __accel_enabled = false;
    bool __gyro_enabled = false;
    uint32_t revisionID;
    uint8_t  usid[6];
    bool __gDataReady = false;
    bool __aDataReady = false;
    int __irq = -1;
    uint8_t __irq_enable_mask = false;
    uint8_t __fifo_mode;
    bool __fifo_interrupt = false;;
    uint8_t *__fifo_buffer = NULL;
    uint16_t __fifo_size = 0;

    EventCallBack_t eventWomEvent = NULL;
    EventCallBack_t eventTagEvent = NULL;
    EventCallBack_t eventPedometerEvent = NULL;
    EventCallBack_t eventNoMotionEvent = NULL;
    EventCallBack_t eventAnyMotionEvent = NULL;
    EventCallBack_t eventSignificantMotion = NULL;
    EventCallBack_t eventGyroDataReady = NULL;
    EventCallBack_t eventAccelDataReady = NULL;
    EventCallBack_t eventDataLocking = NULL;


    int writeCommand(CommandTable cmd, uint32_t wait_ms = 1000)
    {
        int      val;
        uint32_t startMillis;
        if (writeRegister(QMI8658_REG_CTRL9, cmd) == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }
        startMillis = millis();
        do {
            val = readRegister(QMI8658_REG_STATUS_INT);
            delay(1);
            if (millis() - startMillis > wait_ms) {
                log_e("wait for ctrl9 command done time out : %d val:%d", cmd, val);
                return DEV_WIRE_TIMEOUT;
            }
        } while (val != DEV_WIRE_ERR && !(val & 0x80));

        if (writeRegister(QMI8658_REG_CTRL9, CTRL_CMD_ACK) == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }

        startMillis = millis();
        do {
            val = readRegister(QMI8658_REG_STATUS_INT);
            delay(1);
            if (millis() - startMillis > wait_ms) {
                log_e("Clear ctrl9 command done flag timeout : %d val:%d", cmd, val);
                return DEV_WIRE_TIMEOUT;
            }
        } while (val != DEV_WIRE_ERR && (val & 0x80));

        return DEV_WIRE_NONE;
    }



    uint8_t mgToBytes(float mg)
    {
        float g = mg / 1000.0;      // Convert to grams
        int units = (int)round(g / 0.03125); //Convert grams to units of specified(1/32) resolution
        return (units & 0x1F) << 3; // Shift the 5 decimal places to the left by 3 places, because there are only 3 integer places
    }


protected:

    bool initImpl()
    {
        uint8_t buffer[6] = {0};


        if (__irq != -1) {
            this->setGpioMode(__irq, INPUT);
        }

        if (!reset()) {
            return false;
        }

        uint8_t id = whoAmI();
        if (id != QMI8658_REG_WHOAMI_DEFAULT) {
            log_e("ERROR! ID NOT MATCH QMI8658 , Response id is 0x%x", id);
            return false;
        }
        // Enable address auto increment, Big-Endian format
        // writeRegister(QMI8658_REG_CTRL1, 0x60);

        // Little-Endian / address auto increment
        // writeRegister(QMI8658_REG_CTRL1, 0x40);

        // no need . reset function has set
        //EN.ADDR_AI
        // setRegisterBit(QMI8658_REG_CTRL1, 6);

        // Use STATUS_INT.bit7 as CTRL9 handshake
        writeRegister(QMI8658_REG_CTRL8, 0x80);

        // Get firmware version and usid
        writeCommand(CTRL_CMD_COPY_USID);

        if (readRegister(QMI8658_REG_DQW_L, buffer, 3) != DEV_WIRE_ERR) {
            revisionID = buffer[0] | (uint32_t)(buffer[1] << 8) | (uint32_t)(buffer[2] << 16);
            log_d("FW Version :0x%02X%02X%02X", buffer[0], buffer[1], buffer[2]);
        }

        if (readRegister(QMI8658_REG_DVX_L, usid, 6) != DEV_WIRE_ERR) {
            log_d("USID :%02X%02X%02X%02X%02X%02X",
                  usid[0], usid[1], usid[2],
                  usid[3], usid[4], usid[5]);
        }

        return true;
    }

    int getReadMaskImpl()
    {
        return 0x80;
    }

};








