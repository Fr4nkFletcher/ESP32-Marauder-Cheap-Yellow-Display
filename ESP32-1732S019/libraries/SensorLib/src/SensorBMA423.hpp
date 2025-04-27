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
 * @file      SensorBMA423.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-03-31
 * @note      Most source code references come from the https://github.com/boschsensortec/BMA423-Sensor-API
 *            Simplification for Arduino
 */
#pragma once

#include "REG/BMA423Constants.h"
#include "SensorCommon.tpp"

class SensorBMA423 :
    public SensorCommon<SensorBMA423>
{
    friend class SensorCommon<SensorBMA423>;
public:

    enum AccelRange {
        RANGE_2G,
        RANGE_4G,
        RANGE_8G,
        RANGE_16G
    };

    /*! Output data rate in Hz */
    enum AccelODR {
        ODR_0_78HZ = 1,
        ODR_1_56HZ,
        ODR_3_12HZ,
        ODR_6_25HZ,
        ODR_12_5HZ,
        ODR_25HZ,
        ODR_50HZ,
        ODR_100HZ,
        ODR_200HZ,
        ODR_400HZ,
        ODR_800HZ,
        ODR_1600HZ,
    };

    /*! Bandwidth parameter, determines filter configuration */
    enum BandWidth {
        BW_OSR4_AVG1,
        BW_OSR2_AVG2,
        BW_NORMAL_AVG4,
        BW_CIC_AVG8,
        BW_RES_AVG16,
        BW_RES_AVG32,
        BW_RES_AVG64,
        BW_RES_AVG128,
    };

    enum PerformanceMode {
        PERF_CIC_AVG_MODE,
        PERF_CONTINUOUS_MODE,
    };

    enum TemperatureUnit {
        TEMP_DEG,
        TEMP_FAHREN,
        TEMP_KELVIN,
    };

    // Calculate direction facing the front of the chip
    enum SensorDir {
        DIRECTION_BOTTOM_LEFT,
        DIRECTION_TOP_RIGHT,
        DIRECTION_TOP_LEFT,
        DIRECTION_BOTTOM_RIGHT,
        DIRECTION_BOTTOM,
        DIRECTION_TOP
    };

    // Chip orientation and orientation
    enum SensorRemap {
        // Top right corner
        REMAP_TOP_LAYER_RIGHT_CORNER,
        // Front bottom left corner
        REMAP_TOP_LAYER_BOTTOM_LEFT_CORNER,
        // Top left corner
        REMAP_TOP_LAYER_LEFT_CORNER,
        // Top bottom right corner
        REMAP_TOP_LAYER_BOTTOM_RIGHT_CORNER,
        // Bottom top right corner
        REMAP_BOTTOM_LAYER_TOP_RIGHT_CORNER,
        // Bottom bottom left corner
        REMAP_BOTTOM_LAYER_BOTTOM_LEFT_CORNER,
        // Bottom bottom right corner
        REMAP_BOTTOM_LAYER_BOTTOM_RIGHT_CORNER,
        // Bottom top left corner
        REMAP_BOTTOM_LAYER_TOP_LEFT_CORNER,
    };

    enum Feature {
        /**\name Feature enable macros for the sensor */
        FEATURE_STEP_CNTR   = 0x01,
        /**\name Below macros are mutually exclusive */
        FEATURE_ANY_MOTION  = 0x02,
        FEATURE_NO_MOTION   = 0x04,
        FEATURE_ACTIVITY    = 0x08,
        FEATURE_TILT        = 0x10,
        FEATURE_WAKEUP      = 0x20,
    };

    /**\name Interrupt status macros */
    enum FeatureInterrupt {
        INT_STEP_CNTR     = 0x02,
        INT_ACTIVITY      = 0x04,
        INT_TILT          = 0x05,
        INT_WAKEUP        = 0x20,
        INT_ANY_NO_MOTION = 0x40,
    };

#if defined(ARDUINO)
    SensorBMA423(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BMA423_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    SensorBMA423()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = BMA423_SLAVE_ADDRESS;
    }

    ~SensorBMA423()
    {
        deinit();
    }


#if defined(ARDUINO)
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BMA423_SLAVE_ADDRESS)
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }
#endif


    void deinit()
    {
        // end();
    }

    void reset()
    {
        writeRegister(BMA423_RESET_REG, 0xB6);
        delay(20);
    }

    bool enablePowerSave()
    {
        uint8_t val;

        val = readRegister(BMA4_POWER_CONF_ADDR);
        val |= BMA4_ADVANCE_POWER_SAVE_MSK;
        writeRegister(BMA4_POWER_CONF_ADDR, val);
        return true;
    }

    bool disablePowerSave()
    {
        uint8_t val;
        val = readRegister(BMA4_POWER_CONF_ADDR);
        val &= ~(BMA4_ADVANCE_POWER_SAVE_MSK);
        writeRegister(BMA4_POWER_CONF_ADDR, val);
        return true;
    }

    void disableInterruptCtrl()
    {
        writeRegister(BMA4_INIT_CTRL_ADDR, 0x00);
    }

    void enableInterruptCtrl()
    {
        writeRegister(BMA4_INIT_CTRL_ADDR, 0x01);
    }


    bool enableAccelerometer()
    {
        uint8_t val;
        val = readRegister(BMA4_POWER_CTRL_ADDR);
        val |= BMA4_ACCEL_ENABLE_MSK;
        writeRegister(BMA4_POWER_CTRL_ADDR, val);
        return true;
    }

    bool disableAccelerometer()
    {
        uint8_t val;
        val = readRegister( BMA4_POWER_CTRL_ADDR);
        val &= (~BMA4_ACCEL_ENABLE_MSK);
        writeRegister(BMA4_POWER_CTRL_ADDR, val);
        return true;
    }

    bool configAccelerometer(AccelRange range = RANGE_4G, AccelODR odr = ODR_200HZ,
                             BandWidth bw = BW_NORMAL_AVG4,
                             PerformanceMode perfMode = PERF_CONTINUOUS_MODE)

    {
        uint8_t buffer[2] = {0, 0};
        if (perfMode == PERF_CONTINUOUS_MODE) {
            if (bw > BW_NORMAL_AVG4) {
                return false;
            }
        } else if (perfMode == PERF_CIC_AVG_MODE) {
            if (bw > BW_RES_AVG128) {
                return false;
            }
        } else {
            return false;
        }
        if ((odr < ODR_0_78HZ) || (odr > ODR_1600HZ)) {
            return false;
        }

        buffer[0] = odr & 0x0F;
        buffer[0] |= (uint8_t)(bw << 4);
        buffer[0] |= (uint8_t)(perfMode << 7);
        buffer[1] = range & 0x03;

        /* Burst write is not possible in
        suspend mode hence individual write is
        used with delay of 1 ms */
        writeRegister(BMA4_ACCEL_CONFIG_ADDR, buffer[0]);
        delay(2);
        writeRegister(BMA4_ACCEL_CONFIG_ADDR + 1, buffer[1]);
        return true;
    }


    bool getAccelRaw(int16_t *rawBuffer)
    {
        uint8_t buffer[6] = {0};
        if (readRegister(BMA4_DATA_8_ADDR, buffer, 6) != DEV_WIRE_ERR) {
            /* Accel data x axis */
            rawBuffer[0] = (int16_t)(buffer[1] << 8) | (buffer[0]);
            /* Accel data y axis */
            rawBuffer[1] = (int16_t)(buffer[3] << 8) | (buffer[2]);
            /* Accel data z axis */
            rawBuffer[2] = (int16_t)(buffer[5] << 8) | (buffer[4]);
        } else {
            return false;
        }
        return true;
    }

    bool getAccelerometer(int16_t &x, int16_t &y, int16_t &z)
    {
        int16_t raw[3];
        if (getAccelRaw(raw)) {
            x = raw[0] / 16;
            y = raw[1] / 16;
            z = raw[2] / 16;
            return true;
        }
        return false;
    }

    float getTemperature(TemperatureUnit unit)
    {
        int32_t raw = readRegister(BMA4_TEMPERATURE_ADDR);
        /* '0' value read from the register corresponds to 23 degree C */
        raw = (raw * 1000) + (23 * 1000);
        switch (unit) {
        case TEMP_FAHREN:
            /* Temperature in degree Fahrenheit */
            /* 1800 = 1.8 * 1000 */
            raw = ((raw / 1000) * 1800) + (32 * 1000);
            break;
        case TEMP_KELVIN:
            /* Temperature in degree Kelvin */
            /* 273150 = 273.15 * 1000 */
            raw = raw + 273150;
            break;
        default:
            break;
        }
        float res = (float)raw / (float)1000.0;
        /* 0x80 - raw read from the register and 23 is the ambient raw added.
         * If the raw read from register is 0x80, it means no valid
         * information is available */
        if (((raw - 23) / 1000) == 0x80) {
            return 0;
        }
        return res;
    }


    uint8_t direction()
    {
        int16_t x = 0, y = 0, z = 0;
        getAccelerometer(x, y, z);
        uint16_t absX = abs(x);
        uint16_t absY = abs(y);
        uint16_t absZ = abs(z);
        if ((absZ > absX) && (absZ > absY)) {
            if (z > 0) {
                return  DIRECTION_TOP;
            } else {
                return DIRECTION_BOTTOM;
            }
        } else if ((absY > absX) && (absY > absZ)) {
            if (y > 0) {
                return DIRECTION_TOP_RIGHT;
            } else {
                return  DIRECTION_BOTTOM_LEFT;
            }
        } else {
            if (x < 0) {
                return  DIRECTION_BOTTOM_RIGHT;
            } else {
                return DIRECTION_TOP_LEFT;
            }
        }
        return 0;
    }

    bool setRemapAxes(SensorRemap remap)
    {
        //Top
        // No.1 REG: 0x3e -> 0x88   REG: 0x3f -> 0x0
        // No.2 REG: 0x3e -> 0xac   REG: 0x3f -> 0x0
        // No.3 REG: 0x3e -> 0x85   REG: 0x3f -> 0x0
        // No.4 REG: 0x3e -> 0xa1   REG: 0x3f -> 0x0

        // Bottom
        // No.5 REG: 0x3e -> 0x81   REG: 0x3f -> 0x1
        // No.6 REG: 0x3e -> 0xa5   REG: 0x3f -> 0x1
        // No.7 REG: 0x3e -> 0x8c   REG: 0x3f -> 0x1
        // No.8 REG: 0x3e -> 0xa8   REG: 0x3f -> 0x1

        uint8_t configReg0[] = {0x88, 0xAC, 0x85, 0xA1, 0x81, 0xA5, 0x8C, 0xA8};
        if (remap > sizeof(configReg0) / sizeof(configReg0[0])) {
            return false;
        }
        uint8_t buffer[BMA423_FEATURE_SIZE] = {0};
        uint8_t index = BMA423_AXES_REMAP_OFFSET;
        if (readRegister(BMA4_FEATURE_CONFIG_ADDR,  buffer, BMA423_FEATURE_SIZE) == DEV_WIRE_ERR) {
            return false;
        }
        buffer[index] = configReg0[remap];
        buffer[index + 1] = remap >= 4 ? 0x00 : 0x01;
        return writeRegister(BMA4_FEATURE_CONFIG_ADDR,  buffer, BMA423_FEATURE_SIZE) != DEV_WIRE_ERR;
    }




    bool setStepCounterWatermark(uint16_t watermark)
    {
        uint8_t buffer[BMA423_FEATURE_SIZE] = {0};
        uint8_t index = BMA423_STEP_CNTR_OFFSET;
        uint16_t wm_lsb = 0;
        uint16_t wm_msb = 0;
        int rslt;
        uint16_t data = 0;

        rslt = readRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, BMA423_FEATURE_SIZE);
        if (rslt != DEV_WIRE_ERR) {
            wm_lsb = buffer[index];
            wm_msb = buffer[index + 1] << 8;
            data = wm_lsb | wm_msb;
            /* Sets only watermark bits in the complete 16 bits of data */
            data = ((data & ~(0x03FFU)) | (watermark & 0x03FFU));
            /* Splits 16 bits of data to individual 8 bits data */
            buffer[index] = (uint8_t)(data & 0x00FFU);
            buffer[index + 1] = (uint8_t)((data & 0xFF00U) >> 8);
            /* Writes stepcounter watermark settings in the sensor */
            rslt = writeRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, BMA423_FEATURE_SIZE);
        }
        return rslt != DEV_WIRE_ERR;
    }

    bool disablePedometer()
    {
        return enablePedometer(false);
    }

    bool enablePedometer(bool enable = true)
    {
        int rslt;
        uint8_t buffer[BMA423_FEATURE_SIZE] = {0};
        /* Step detector enable bit pos. is 1 byte ahead of the base address */
        uint8_t index = BMA423_STEP_CNTR_OFFSET + 1;
        rslt = readRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, BMA423_FEATURE_SIZE);
        if (rslt != DEV_WIRE_ERR) {
            buffer[index] = ((buffer[index] & ~0x08) | ((enable << 3) & 0x08));
            rslt = writeRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, BMA423_FEATURE_SIZE);
        }
        return rslt != DEV_WIRE_ERR;
    }

    uint32_t getPedometerCounter()
    {
        uint8_t buffer[4] = {0};
        /* Reads the step counter output data from the gpio register */
        int rslt = readRegister(BMA4_STEP_CNT_OUT_0_ADDR, buffer, 4);
        if (rslt != DEV_WIRE_ERR) {
            return ((uint32_t)buffer[0]) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
        }
        return DEV_WIRE_NONE;
    }

    void resetPedometer()
    {
        uint8_t buffer[BMA423_FEATURE_SIZE] = {0};
        /* Reset bit is 1 byte ahead of base address */
        uint8_t index = BMA423_STEP_CNTR_OFFSET + 1;
        int rslt = readRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, BMA423_FEATURE_SIZE);
        if (rslt != DEV_WIRE_ERR) {
            buffer[index] = ((buffer[index] & ~0x04U) | ((1 << 2U) & 0x04U));
            writeRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, BMA423_FEATURE_SIZE);
        }
    }

    bool enableFeature(uint8_t feature, uint8_t enable)
    {
        uint8_t buffer[BMA423_FEATURE_SIZE] = {0};
        int rslt;
        uint8_t len;
        /* Update the length for read and write */
        update_len(&len, feature, enable);
        rslt = readRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, len);
        if (rslt != DEV_WIRE_ERR) {
            if (enable) {
                /* Enables the feature */
                rslt = feature_enable(feature, len, buffer);
            } else {
                /* Disables the feature */
                rslt = feature_disable(feature, len, buffer);
            }
        }
        return rslt != DEV_WIRE_ERR;
    }

    uint16_t readIrqStatus()
    {
        uint8_t data[2] = {0};
        if (readRegister(BMA4_INT_STAT_0_ADDR, data, 2) != DEV_WIRE_ERR) {
            int_status = data[0] | (data[1] << 8);
            return int_status;
        }
        return DEV_WIRE_NONE;
    }

    uint16_t getIrqStatus()
    {
        return int_status;
    }

    bool configInterrupt(
        /*! Trigger condition of interrupt pin */
        uint8_t edge_ctrl = BMA4_LEVEL_TRIGGER,
        /*! Level of interrupt pin */
        uint8_t level = BMA4_ACTIVE_HIGH,
        /*! Behaviour of interrupt pin to open drain */
        uint8_t od = BMA4_PUSH_PULL,
        /*! Output enable for interrupt pin */
        uint8_t output_en = BMA4_OUTPUT_ENABLE,
        /*! Input enable for interrupt pin */
        uint8_t input_en = BMA4_INPUT_DISABLE,
        /*! Variable used to select the interrupt pin1 or pin2 for interrupt configuration. */
        uint8_t int_line = BMA4_INTR1_MAP
    )
    {
        uint8_t interrupt_address_array[2] = {BMA4_INT1_IO_CTRL_ADDR, BMA4_INT2_IO_CTRL_ADDR};
        uint8_t data = 0;
        if (int_line > 1) {
            return false;
        }
        data = ((uint8_t)((edge_ctrl & BMA4_INT_EDGE_CTRL_MASK) |
                          ((level << 1) & BMA4_INT_LEVEL_MASK) |
                          ((od << 2) & BMA4_INT_OPEN_DRAIN_MASK) |
                          ((output_en << 3) & BMA4_INT_OUTPUT_EN_MASK) |
                          ((input_en << 4) & BMA4_INT_INPUT_EN_MASK)));

        this->int_line = int_line;

        return  writeRegister(interrupt_address_array[int_line],  &data, 1) != DEV_WIRE_ERR;
    }

    bool configFeatureInterrupt(uint16_t  feature_interrupt_mask, bool enable)
    {
        return  interruptMap(int_line,  feature_interrupt_mask, enable);
    }


    bool enablePedometerIRQ()
    {
        return  (interruptMap(int_line,  INT_STEP_CNTR, true));
    }

    bool enableTiltIRQ()
    {
        return  (interruptMap(int_line, INT_TILT, true));
    }

    bool enableWakeupIRQ()
    {
        return  (interruptMap(int_line, INT_WAKEUP, true));
    }

    bool enableAnyNoMotionIRQ()
    {
        return  (interruptMap(int_line, INT_ANY_NO_MOTION, true));
    }

    bool enableActivityIRQ()
    {
        return  (interruptMap(int_line, INT_ACTIVITY, true));
    }

    bool disablePedometerIRQ()
    {
        return  (interruptMap(int_line,  INT_STEP_CNTR, false));
    }

    bool disableTiltIRQ()
    {
        return  (interruptMap(int_line, INT_TILT, false));
    }

    bool disableWakeupIRQ()
    {
        return  (interruptMap(int_line, INT_WAKEUP, false));
    }

    bool disableAnyNoMotionIRQ()
    {
        return  (interruptMap(int_line, INT_ANY_NO_MOTION, false));
    }

    bool disableActivityIRQ()
    {
        return  (interruptMap(int_line, INT_ACTIVITY, false));
    }


    inline bool isActivity()
    {
        return (int_status & BMA423_ACTIVITY_INT);
    }

    inline bool isTilt()
    {
        return (int_status & BMA423_TILT_INT);
    }

    inline bool isDoubleTap()
    {
        return (int_status & BMA423_WAKEUP_INT);
    }

    inline bool isAnyNoMotion()
    {
        return (int_status & BMA423_ACTIVITY_INT);
    }

    inline bool isPedometer()
    {
        return (int_status & BMA423_STEP_CNTR_INT);
    }

private:

    bool interruptMap(uint8_t int_line, uint16_t int_map, uint8_t enable)
    {
        int rslt;
        uint8_t data[3] = {0, 0, 0};
        uint8_t index[2] = {BMA4_INT_MAP_1_ADDR, BMA4_INT_MAP_2_ADDR};
        rslt = readRegister(BMA4_INT_MAP_1_ADDR, data, 3);
        if (enable) {
            /* Feature interrupt mapping */
            data[int_line] |= (uint8_t)(int_map & (0x00FF));
            /* Hardware interrupt mapping */
            if (int_line == BMA4_INTR2_MAP)
                data[2] |= (uint8_t)((int_map & (0xFF00)) >> 4);
            else
                data[2] |= (uint8_t)((int_map & (0xFF00)) >> 8);

            rslt = writeRegister(index[int_line], &data[int_line], 1);
            rslt = writeRegister(BMA4_INT_MAP_DATA_ADDR, &data[2], 1);

        } else {
            /* Feature interrupt un-mapping */
            data[int_line] &= (~(uint8_t)(int_map & (0x00FF)));
            /* Hardware interrupt un-mapping */
            if (int_line == BMA4_INTR2_MAP)
                data[2] &= (~(uint8_t)((int_map & (0xFF00)) >> 4));
            else
                data[2] &= (~(uint8_t)((int_map & (0xFF00)) >> 8));

            rslt = writeRegister(index[int_line], &data[int_line], 1);
            rslt = writeRegister(BMA4_INT_MAP_DATA_ADDR, &data[2], 1);

        }
        return rslt != DEV_WIRE_ERR;
    }

    /*!
     *  @brief This API sets the interrupt mode in the sensor.
     */
    bool setInterruptMode(uint8_t mode)
    {
        if (mode == BMA4_NON_LATCH_MODE || mode == BMA4_LATCH_MODE)
            return writeRegister(BMA4_INTR_LATCH_ADDR, &mode, 1) != DEV_WIRE_ERR;
        return false;
    }


    bool feature_disable(uint8_t feature, uint8_t len, uint8_t *buffer)
    {
        uint8_t index = 0;

        /* Disable step counter */
        if ((feature & FEATURE_STEP_CNTR) > 0) {
            /* Step counter enable bit pos. is 1 byte ahead of the
            base address */
            index = BMA423_STEP_CNTR_OFFSET + 1;
            buffer[index] = buffer[index] & (~BMA423_STEP_CNTR_EN_MSK);
        }

        /* Disable activity */
        if ((feature & FEATURE_ACTIVITY) > 0) {
            /* Activity enable bit pos. is 1 byte ahead of the
            base address */
            index = BMA423_STEP_CNTR_OFFSET + 1;
            buffer[index] = buffer[index] & (~BMA423_ACTIVITY_EN_MSK);
        }
        /* Disable tilt */
        if ((feature & FEATURE_TILT) > 0) {
            /* Tilt enable bit pos. is the base address(0x3A) of tilt */
            index = BMA423_TILT_OFFSET;
            buffer[index] = buffer[index] & (~BMA423_TILT_EN_MSK);
        }

        /* Disable wakeup */
        if ((feature & FEATURE_WAKEUP) > 0) {
            /* Tilt enable bit pos. is the base address(0x38) of wakeup */
            index = BMA423_WAKEUP_OFFSET;
            buffer[index] = buffer[index] & (~BMA423_WAKEUP_EN_MSK);
        }

        /* Disable anymotion/nomotion */
        if ((feature & FEATURE_ANY_MOTION) > 0 || (feature & FEATURE_NO_MOTION) > 0) {
            /* Any/Nomotion enable bit pos. is 1 bytes ahead of the
            any/nomotion base address(0x00) */
            index = 1;

            if ((feature & FEATURE_ANY_MOTION) > 0) {
                /* Disable anymotion */
                buffer[index] = buffer[index] | BMA423_ANY_NO_MOTION_SEL_MSK;
            } else {
                /* Disable nomotion */
                buffer[index] = buffer[index] & (~BMA423_ANY_NO_MOTION_SEL_MSK);
            }
            /* Any/Nomotion axis enable bit pos. is 3 byte ahead of the
            any/nomotion base address(0x00) */
            index = 3;
            buffer[index] = buffer[index] & (~BMA423_ANY_NO_MOTION_AXIS_EN_MSK);
        }
        /* Write the configured settings in the sensor */
        return writeRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, len) != DEV_WIRE_ERR;
    }

    int feature_enable(uint8_t feature, uint8_t len, uint8_t *buffer)
    {
        uint8_t index = 0;

        /* Enable step counter */
        if ((feature & FEATURE_STEP_CNTR) > 0) {
            /* Step counter enable bit pos. is 1 byte ahead of the
            base address */
            index = BMA423_STEP_CNTR_OFFSET + 1;
            buffer[index] = buffer[index] | BMA423_STEP_CNTR_EN_MSK;
        }

        /* Enable activity */
        if ((feature & FEATURE_ACTIVITY) > 0) {
            /* Activity enable bit pos. is 1 byte ahead of the
            base address */
            index = BMA423_STEP_CNTR_OFFSET + 1;
            buffer[index] = buffer[index] | BMA423_ACTIVITY_EN_MSK;
        }
        /* Enable tilt */
        if ((feature & FEATURE_TILT) > 0) {
            /* Tilt enable bit pos. is the base address(0x3A) of tilt */
            index = BMA423_TILT_OFFSET;
            buffer[index] = buffer[index] | BMA423_TILT_EN_MSK;
        }

        /* Enable wakeup */
        if ((feature & FEATURE_WAKEUP) > 0) {
            /* Wakeup enable bit pos. is the base address(0x38) of wakeup */
            index = BMA423_WAKEUP_OFFSET;
            buffer[index] = buffer[index] | BMA423_WAKEUP_EN_MSK;
        }

        /* Enable anymotion/nomotion */
        if ((feature & FEATURE_ANY_MOTION) > 0 || (feature & FEATURE_NO_MOTION) > 0) {
            /* Any/Nomotion enable bit pos. is 1 bytes ahead of the
            any/nomotion base address(0x00) */
            index = 1;

            if ((feature & FEATURE_ANY_MOTION) > 0) {
                /* Enable anymotion */
                buffer[index] = buffer[index] & (~BMA423_ANY_NO_MOTION_SEL_MSK);
            } else {
                /* Enable nomotion */
                buffer[index] = buffer[index] | BMA423_ANY_NO_MOTION_SEL_MSK;
            }
        }

        /* Write the feature enable settings in the sensor */
        return writeRegister(BMA4_FEATURE_CONFIG_ADDR, buffer, len) != DEV_WIRE_ERR;
    }

    void update_len(uint8_t *len, uint8_t feature, uint8_t enable)
    {
        uint8_t length = BMA423_FEATURE_SIZE;

        if ((feature == FEATURE_ANY_MOTION) || (feature == FEATURE_NO_MOTION)) {
            /* Change the feature length to 2 for reading and writing of 2 bytes for
            any/no-motion enable */
            length = BMA423_ANYMOTION_EN_LEN;

            /* Read and write 4 byte to disable the any/no motion completely along with
            all axis */
            if (!enable) {
                /*Change the feature length to 4 for reading and writing
                of 4 bytes for any/no-motion enable */
                length = length + 2;
            }
        }
        *len = length;
    }

    bool configure()
    {
        uint8_t val;
        val = readRegister(BMA4_INTERNAL_STAT);
        if (val == BMA4_ASIC_INITIALIZED) {
            log_d("No need configure!");
            readIrqStatus();    //clear irq status
            return true;
        }

        disablePowerSave();
        delay(1);
        disableInterruptCtrl();

        const uint8_t *stream_data = bma423_config_file;
        const uint8_t maxReadWriteLenght = 32;

        for (size_t index = 0; index < BMA4_CONFIG_STREAM_SIZE; index += maxReadWriteLenght) {
            uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
            uint8_t asic_lsb = ((index / 2) & 0x0F);
            writeRegister(BMA4_RESERVED_REG_5B_ADDR,  asic_lsb);
            writeRegister(BMA4_RESERVED_REG_5C_ADDR,  asic_msb);
            writeRegister(BMA4_FEATURE_CONFIG_ADDR,  (uint8_t *)(stream_data + index), maxReadWriteLenght);
        }

        enableInterruptCtrl();
        delay(150);
        val = readRegister(BMA4_INTERNAL_STAT);
        if (val == BMA4_ASIC_INITIALIZED) {
            log_d("BMA configure SUCCESS!");
        } else {
            log_d("BMA configure FAILED!");
        }
        return val == BMA4_ASIC_INITIALIZED;
    }

    bool initImpl()
    {
        uint8_t id;
        int retry = 2;

        while (retry--) {
            id = readRegister(BMA4_CHIP_ID_ADDR);
            if (id != BMA423_CHIP_ID) {
                reset();
            }
        }

        if (id == BMA423_CHIP_ID) {
            return configure();
        }

        log_d("ChipID:0x%x should be 0x%x", id, BMA423_CHIP_ID);
        return false;

    }


    int getReadMaskImpl()
    {
        return DEV_WIRE_ERR;
    }

    uint16_t int_status;
    uint8_t int_line;

protected:


};



