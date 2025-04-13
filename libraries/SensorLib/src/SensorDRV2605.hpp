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
 * @file      SensorDRV2605.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-03
 * @note      Source code from https://github.com/adafruit/Adafruit_DRV2605_Library
 */
#pragma once

#include "REG/DRV2605Constants.h"
#include "SensorCommon.tpp"

class SensorDRV2605 :
    public SensorCommon<SensorDRV2605>
{
    friend class SensorCommon<SensorDRV2605>;
public:

#if defined(ARDUINO)
    SensorDRV2605(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = DRV2605_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    SensorDRV2605()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = DRV2605_SLAVE_ADDRESS;
    }

    ~SensorDRV2605()
    {
        deinit();
    }

#if defined(ARDUINO)
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = DRV2605_SLAVE_ADDRESS)
    {
        return SensorCommon::begin(w, addr, sda, scl);
    }
#endif


    void deinit()
    {
        // end();
    }

    /**************************************************************************/
    /*!
      @brief Select the haptic waveform to use.
      @param slot The waveform slot to set, from 0 to 7
      @param w The waveform sequence value, refers to an index in the ROM library.

        Playback starts at slot 0 and continues through to slot 7, stopping if it
      encounters a value of 0. A list of available waveforms can be found in
      section 11.2 of the datasheet: http://www.adafruit.com/datasheets/DRV2605.pdf
    */
    /**************************************************************************/
    void setWaveform(uint8_t slot, uint8_t w)
    {
        writeRegister(DRV2605_REG_WAVESEQ1 + slot, w);
    }

    /**************************************************************************/
    /*!
      @brief Select the waveform library to use.
      @param lib Library to use, 0 = Empty, 1-5 are ERM, 6 is LRA.

        See section 7.6.4 in the datasheet for more details:
      http://www.adafruit.com/datasheets/DRV2605.pdf
    */
    /**************************************************************************/
    void selectLibrary(uint8_t lib)
    {
        writeRegister(DRV2605_REG_LIBRARY, lib);
    }

    /**************************************************************************/
    /*!
      @brief Start playback of the waveforms (start moving!).
    */
    /**************************************************************************/
    void run()
    {
        writeRegister(DRV2605_REG_GO, 1);
    }

    /**************************************************************************/
    /*!
      @brief Stop playback.
    */
    /**************************************************************************/
    void stop()
    {
        writeRegister(DRV2605_REG_GO, 0);
    }

    /**************************************************************************/
    /*!
      @brief Set the device mode.
      @param mode Mode value, see datasheet section 7.6.2:
      http://www.adafruit.com/datasheets/DRV2605.pdf

        0: Internal trigger, call run() to start playback\n
        1: External trigger, rising edge on IN pin starts playback\n
        2: External trigger, playback follows the state of IN pin\n
        3: PWM/analog input\n
        4: Audio\n
        5: Real-time playback\n
        6: Diagnostics\n
        7: Auto calibration
    */
    /**************************************************************************/
    void setMode(uint8_t mode)
    {
        writeRegister(DRV2605_REG_MODE, mode);
    }

    /**************************************************************************/
    /*!
      @brief Set the realtime value when in RTP mode, used to directly drive the
      haptic motor.
      @param rtp 8-bit drive value.
    */
    /**************************************************************************/
    void setRealtimeValue(uint8_t rtp)
    {
        writeRegister(DRV2605_REG_RTPIN, rtp);
    }

    /**************************************************************************/
    /*!
      @brief Use ERM (Eccentric Rotating Mass) mode.
    */
    /**************************************************************************/
    void useERM()
    {
        writeRegister(DRV2605_REG_FEEDBACK, readRegister(DRV2605_REG_FEEDBACK) & 0x7F);
    }

    /**************************************************************************/
    /*!
      @brief Use LRA (Linear Resonance Actuator) mode.
    */
    /**************************************************************************/
    void useLRA()
    {
        writeRegister(DRV2605_REG_FEEDBACK, readRegister(DRV2605_REG_FEEDBACK) | 0x80);
    }

private:
    bool initImpl()
    {
        int chipID = readRegister(DRV2605_REG_STATUS);
        if (chipID == -1) {
            return false;
        }
        chipID >>= 5;

        if (chipID != DRV2604_CHIP_ID &&
                chipID != DRV2605_CHIP_ID &&
                chipID != DRV2604L_CHIP_ID &&
                chipID != DRV2605L_CHIP_ID ) {
            LOG("ChipID:0x%x should be 0x03 or 0x04 or 0x06 or 0x07\n", chipID);
            return false;
        }

        writeRegister(DRV2605_REG_MODE, 0x00); // out of standby

        writeRegister(DRV2605_REG_RTPIN, 0x00); // no real-time-playback

        writeRegister(DRV2605_REG_WAVESEQ1, 1); // strong click
        writeRegister(DRV2605_REG_WAVESEQ2, 0); // end sequence

        writeRegister(DRV2605_REG_OVERDRIVE, 0); // no overdrive

        writeRegister(DRV2605_REG_SUSTAINPOS, 0);
        writeRegister(DRV2605_REG_SUSTAINNEG, 0);
        writeRegister(DRV2605_REG_BREAK, 0);
        writeRegister(DRV2605_REG_AUDIOMAX, 0x64);

        // ERM open loop

        // turn off N_ERM_LRA
        writeRegister(DRV2605_REG_FEEDBACK,
                      readRegister(DRV2605_REG_FEEDBACK) & 0x7F);
        // turn on ERM_OPEN_LOOP
        writeRegister(DRV2605_REG_CONTROL3,
                      readRegister(DRV2605_REG_CONTROL3) | 0x20);

        return true;
    }

    int getReadMaskImpl()
    {
        return -1;
    }

protected:

};



