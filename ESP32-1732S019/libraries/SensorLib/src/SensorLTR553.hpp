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
 * @file      SensorLTR553ALS.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-09
 *
 */
#pragma once

#include "REG/LTR533Constants.h"
#include "SensorCommon.tpp"


class SensorLTR553 :
    public SensorCommon<SensorLTR553>
{
    friend class SensorCommon<SensorLTR553>;
public:

    enum IrqLevel {
        ALS_IRQ_ACTIVE_LOW, // INT pin is considered active when it is a logic 0 (default)
        ALS_IRQ_ACTIVE_HIGH // INT pin is considered active when it is a logic 1
    };
    enum IrqMode {
        ALS_IRQ_ONLY_PS = 1,    // Only PS measurement can trigger interrupt
        ALS_IRQ_ONLY_ALS,   // Only ALS measurement can trigger interrupt
        ALS_IRQ_BOTH,       // Both ALS and PS measurement can trigger interrupt
    };

    enum LightSensorGain {
        ALS_GAIN_1X  = 0x00,    // 1 lux to 64k lux (default)
        ALS_GAIN_2X  = 0x01,    // 0.5 lux to 32k lux
        ALS_GAIN_4X  = 0x02,    // 0.25 lux to 16k lux
        ALS_GAIN_8X  = 0x03,    // 0.125 lux to 8k lux
        ALS_GAIN_48X = 0x06,    // 0.02 lux to 1.3k lux
        ALS_GAIN_96X = 0x07,    // 0.01 lux to 600 lux
    };

    enum PsLedPeriod {
        PS_LED_PLUSE_30KHZ = 0x00,
        PS_LED_PLUSE_40KHZ,
        PS_LED_PLUSE_50KHZ,
        PS_LED_PLUSE_60KHZ,
        PS_LED_PLUSE_70KHZ,
        PS_LED_PLUSE_80KHZ,
        PS_LED_PLUSE_90KHZ,
        PS_LED_PLUSE_100KHZ,
    };
    enum PsLedDuty {
        PS_LED_DUTY_25 = 0x00,
        PS_LED_DUTY_50,
        PS_LED_DUTY_75,
        PS_LED_DUTY_100,
    };
    enum PsLedCurrent {
        PS_LED_CUR_5MA = 0x00,
        PS_LED_CUR_10MA,
        PS_LED_CUR_20MA,
        PS_LED_CUR_50MA,
        PS_LED_CUR_100MA,
    };
    enum PsRate {
        PS_MEAS_RATE_50MS,
        PS_MEAS_RATE_70MS,
        PS_MEAS_RATE_100MS,
        PS_MEAS_RATE_200MS,
        PS_MEAS_RATE_500MS,
        PS_MEAS_RATE_1000MS,
        PS_MEAS_RATE_2000MS,
        PS_MEAS_RATE_10MS = 8,
    };

    enum IntegrationTime {
        ALS_INTEGRATION_TIME_100MS = 0x00,
        ALS_INTEGRATION_TIME_50MS,
        ALS_INTEGRATION_TIME_200MS,
        ALS_INTEGRATION_TIME_400MS,
        ALS_INTEGRATION_TIME_150MS,
        ALS_INTEGRATION_TIME_250MS,
        ALS_INTEGRATION_TIME_300MS,
        ALS_INTEGRATION_TIME_350MS,
    };

    enum MeasurementRate {
        ALS_MEASUREMENT_TIME_50MS,
        ALS_MEASUREMENT_TIME_100MS,
        ALS_MEASUREMENT_TIME_200MS,
        ALS_MEASUREMENT_TIME_500MS,
        ALS_MEASUREMENT_TIME_1000MS,
        ALS_MEASUREMENT_TIME_2000MS,
    };

#if defined(ARDUINO)
    SensorLTR553(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = LTR553_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    SensorLTR553()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = LTR553_SLAVE_ADDRESS;
    }

    ~SensorLTR553()
    {
        deinit();
    }

#if defined(ARDUINO)
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = LTR553_SLAVE_ADDRESS)
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }
#endif


    void deinit()
    {
        // end();
    }

    void setIRQLevel(IrqLevel  level)
    {
        level ? setRegisterBit(LTR553_REG_INTERRUPT, 2) : clrRegisterBit(LTR553_REG_INTERRUPT, 2);
    }

    void enableIRQ(IrqMode mode)
    {
        writeRegister(LTR553_REG_INTERRUPT, 0xFC, mode);
    }

    void disableIRQ()
    {
        writeRegister(LTR553_REG_INTERRUPT, 0xFC, 0x00);
    }


    //! Light Sensor !
    bool psAvailable()
    {
        return getRegisterBit(LTR553_REG_ALS_PS_STATUS, 0);
    }

    void setLightSensorThreshold(uint16_t low, uint16_t high)
    {
        uint8_t buffer[4] = {
            (uint8_t)(high & 0xFF),
            (uint8_t)((high >> 8) & 0xFF),
            (uint8_t)(low & 0xFF),
            (uint8_t)((low >> 8) & 0xFF),
        };
        writeRegister(LTR553_REG_ALS_THRES_UP_0, buffer, 4);
    }

    // Controls the N number of times the measurement data is outside the range
    // defined by the upper and lower threshold limits before asserting the interrupt.
    void setLightSensorPersists(uint8_t count)
    {
        writeRegister(LTR553_REG_INTERRUPT_PERSIST, 0xF0, count - 1);
    }

    void setLightSensorRate(IntegrationTime integrationTime, MeasurementRate measurementRate)
    {
        writeRegister(LTR553_REG_ALS_MEAS_RATE, 0x00, (integrationTime & 0xF) << 8 | (measurementRate & 0xF));
    }

    void enableLightSensor()
    {
        setRegisterBit(LTR553_REG_ALS_CONTR, 0);
    }

    void disableLightSensor()
    {
        clrRegisterBit(LTR553_REG_ALS_CONTR, 0);
    }

    void setLightSensorGain(LightSensorGain gain)
    {
        writeRegister(LTR553_REG_ALS_CONTR, 0xE3, gain);
    }

    int getLightSensor(uint8_t ch)
    {
        uint8_t buffer[2] = {0};
        // Check ALS Data is Valid
        if (getRegisterBit(LTR553_REG_ALS_PS_STATUS, 7) != false) {
            return 0;
        }
        int val = readRegister(ch == 1 ? LTR553_REG_ALS_DATA_CH1_0 : LTR553_REG_ALS_DATA_CH0_0, buffer, 2);
        if (val == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }
        return buffer[0] | (buffer[1] << 8);
    }

    //!  Proximity sensor !
    // Controls the N number of times the measurement data is outside the range
    // defined by the upper and lower threshold limits before asserting the interrupt.
    void setProximityPersists(uint8_t count)
    {
        writeRegister(LTR553_REG_INTERRUPT_PERSIST, 0x0F, count == 0 ? 0 : count - 1);
    }

    void setProximityThreshold(uint16_t low, uint16_t high)
    {
        writeRegister(LTR553_REG_PS_THRES_UP_0, lowByte(high));
        writeRegister(LTR553_REG_PS_THRES_UP_1, lowByte(high >> 8) & 0x0F);
        writeRegister(LTR553_REG_PS_THRES_LOW_0, lowByte(low));
        writeRegister(LTR553_REG_PS_THRES_LOW_1, lowByte(low >> 8) & 0x0F);
    }

    void setProximityRate(PsRate rate)
    {
        writeRegister(LTR553_REG_PS_MEAS_RATE, 0xF0, rate & 0x0F);
    }

    void enableProximity()
    {
        writeRegister(LTR553_REG_PS_CONTR, 0xF3u, 0x03u);
    }

    void disableProximity()
    {
        writeRegister(LTR553_REG_PS_CONTR, 0xF3u, 0x00u);
    }

    void enablePsIndicator()
    {
        setRegisterBit(LTR553_REG_PS_CONTR, 5);
    }

    void disablePsIndicator()
    {
        clrRegisterBit(LTR553_REG_PS_CONTR, 5);
    }

    int getProximity(bool *saturated = NULL )
    {
        uint8_t buffer[2] = {0};
        int val = readRegister(LTR553_REG_PS_DATA_0, buffer, 2);
        if (val == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }
        if (saturated) {
            *saturated = buffer[1] & 0x80;
        }
        return buffer[0] | (buffer[1] & 0x03);
    }

    void setPsLedPulsePeriod(PsLedPeriod period)
    {
        writeRegister(LTR553_REG_PS_LED, 0x1F, period);
    }

    void setPsLedDutyCycle(PsLedDuty duty)
    {
        writeRegister(LTR553_REG_PS_LED, 0xE7, duty);
    }

    void setPsLedCurrent(PsLedCurrent cur)
    {
        writeRegister(LTR553_REG_PS_LED, 0xF8, cur);
    }

    void setPsLedPulses(uint8_t pulesNum)
    {
        writeRegister(LTR553_REG_PS_N_PULSES, 0xF0, pulesNum & 0x0F);
    }

    int getPartID()
    {
        int val = readRegister(LTR553_REG_PART_ID);
        if (val == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }
        return (val >> 4) & 0x0F;
    }

    int getRevisionID()
    {
        int val = readRegister(LTR553_REG_PART_ID);
        if (val == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }
        return (val) & 0x0F;
    }

    int getManufacturerID()
    {
        // Manufacturer ID (0x05H)
        return readRegister(LTR553_REG_MANUFAC_ID);
    }

    void reset()
    {
        setRegisterBit(LTR553_REG_ALS_CONTR, 1);
    }

private:

    bool initImpl()
    {
        setReadRegisterSendStop(false);
        reset();
        return getManufacturerID() == LTR553_DEFAULT_MAN_ID;
    }

    int getReadMaskImpl()
    {
        return -1;
    }

protected:

};



