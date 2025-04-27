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
 * @file      SensorQMC6310.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#pragma once

#include "REG/QMC6310Constants.h"
#include "SensorCommon.tpp"

class Polar
{
public:
    Polar(): polar(0), Gauss(0), uT(0) {}
    Polar(float polar, float Gauss, float uT): polar(polar), Gauss(Gauss), uT(uT) {}
    float polar;
    float Gauss;
    float uT;
};


class SensorQMC6310 :
    public SensorCommon<SensorQMC6310>
{
    friend class SensorCommon<SensorQMC6310>;
public:


    enum SensorMode {
        MODE_SUSPEND,
        MODE_NORMAL,
        MODE_SINGLE,
        MODE_CONTINUOUS,
    };

    // Unit:Guass
    enum MagRange {
        RANGE_30G,
        RANGE_12G,
        RANGE_8G,
        RANGE_2G,
    };

    enum OutputRate {
        DATARATE_10HZ,
        DATARATE_50HZ,
        DATARATE_100HZ,
        DATARATE_200HZ,
    };

    enum CtrlReg {
        SET_RESET_ON,
        SET_ONLY_ON,
        SET_RESET_OFF,
    };

    enum OverSampleRatio {
        OSR_8,
        OSR_4,
        OSR_2,
        OSR_1,
    };

    enum DownSampleRatio {
        DSR_1,
        DSR_2,
        DSR_4,
        DSR_8,
    };

#if defined(ARDUINO)
    SensorQMC6310(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = QMC6310_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    SensorQMC6310()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __addr = QMC6310_SLAVE_ADDRESS;
    }

    ~SensorQMC6310()
    {
        deinit();
    }

#if defined(ARDUINO)
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = QMC6310_SLAVE_ADDRESS)
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
        writeRegister(QMC6310_REG_CMD2, 0x80);
        delay(10);
        writeRegister(QMC6310_REG_CMD2, 0x00);
    }

    uint8_t getChipID()
    {
        return readRegister(QMC6310_REG_CHIP_ID);
    }


    int getStatus()
    {
        return readRegister(QMC6310_REG_STAT);
    }

    bool isDataReady()
    {
        if (readRegister(QMC6310_REG_STAT) & 0x01) {
            return true;
        }
        LOG("No ready!\n");
        return false;
    }

    bool isDataOverflow()
    {
        if (readRegister(QMC6310_REG_STAT) & 0x02) {
            return true;
        }
        return false;
    }

    int setSelfTest(bool en)
    {
        return en ? setRegisterBit(QMC6310_REG_CMD2, 1)
               : clrRegisterBit(QMC6310_REG_CMD2, 1);
    }

    int setMode(SensorMode m)
    {
        return writeRegister(QMC6310_REG_CMD1, 0xFC, m);
    }

    int setCtrlRegister(CtrlReg c)
    {
        return writeRegister(QMC6310_REG_CMD2, 0xFC, c);
    }

    int setDataOutputRate(OutputRate odr)
    {
        return writeRegister(QMC6310_REG_CMD1, 0xF3, (odr << 2));
    }

    int setOverSampleRate(OverSampleRatio osr)
    {
        return writeRegister(QMC6310_REG_CMD1, 0xCF, (osr << 4));
    }

    int setDownSampleRate(DownSampleRatio dsr)
    {
        return writeRegister(QMC6310_REG_CMD1, 0x3F, (dsr << 6));
    }

    // Define the sign for X Y and Z axis
    int setSign(uint8_t x, uint8_t y, uint8_t z)
    {
        int sign = x + y * 2 + z * 4;
        return writeRegister(QMC6310_REG_SIGN, sign);
    }

    int configMagnetometer(SensorMode mode, MagRange range, OutputRate odr,
                           OverSampleRatio osr, DownSampleRatio dsr)
    {
        if (setMagRange(range) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        if (writeRegister(QMC6310_REG_CMD1, 0xFC, mode) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        if (writeRegister(QMC6310_REG_CMD1, 0xF3, (odr << 2)) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        if (writeRegister(QMC6310_REG_CMD1, 0xCF, (osr << 4)) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        if (writeRegister(QMC6310_REG_CMD1, 0x3F, (dsr << 6)) != DEV_WIRE_NONE) {
            return DEV_WIRE_ERR;
        }
        return DEV_WIRE_NONE;
    }

    int setMagRange(MagRange range)
    {
        switch (range) {
        case RANGE_30G:
            sensitivity = 0.1;
            break;
        case RANGE_12G:
            sensitivity = 0.04;
            break;
        case RANGE_8G:
            sensitivity = 0.026;
            break;
        case RANGE_2G:
            sensitivity = 0.0066;
            break;
        default:
            break;
        }
        return writeRegister(QMC6310_REG_CMD2, 0xF3, (range << 2));
    }

    void setOffset(int x, int y, int z)
    {
        x_offset = x; y_offset = y; z_offset = z;
    }

    int readData()
    {
        uint8_t buffer[6];
        int16_t x, y, z;
        if (readRegister(QMC6310_REG_LSB_DX, buffer,
                         6) != DEV_WIRE_ERR) {
            x = (int16_t)(buffer[1] << 8) | (buffer[0]);
            y = (int16_t)(buffer[3] << 8) | (buffer[2]);
            z = (int16_t)(buffer[5] << 8) | (buffer[4]);

            if (x == 32767) {
                x = -((65535 - x) + 1);
            }
            x = (x - x_offset);
            if (y == 32767) {
                y = -((65535 - y) + 1);
            }
            y = (y - y_offset);
            if (z == 32767) {
                z = -((65535 - z) + 1);
            }
            z = (z - z_offset);

            raw[0] = x;
            raw[1] = y;
            raw[2] = z;

            mag[0] = (float)x * sensitivity;
            mag[1] = (float)y * sensitivity;
            mag[2] = (float)z * sensitivity;
            return DEV_WIRE_NONE;
        }
        return DEV_WIRE_ERR;
    }

    void setDeclination(float dec)
    {
        _declination = dec;
    }

    bool readPolar(Polar &p)
    {
        if (isDataReady()) {
            readData();
            float x = getX();
            float y = getY();
            float z = getZ();
            float angle = (atan2(x, -y) / PI) * 180.0 + _declination;
            angle = _convertAngleToPositive(angle);
            float magnitude = sqrt(x * x + y * y + z * z);
            p = Polar(angle, magnitude * 100, magnitude);
            return true;
        }
        return false;
    }

    int16_t getRawX()
    {
        return raw[0];
    }

    int16_t getRawY()
    {
        return raw[1];
    }

    int16_t getRawZ()
    {
        return raw[2];
    }

    float getX()
    {
        return mag[0];
    }

    float getY()
    {
        return mag[1];
    }

    float getZ()
    {
        return mag[2];
    }

    void getMag(float &x, float &y, float &z)
    {
        x = mag[0];
        y = mag[1];
        z = mag[2];
    }

    void dumpCtrlRegister()
    {
        uint8_t buffer[2];
        readRegister(QMC6310_REG_CMD1, buffer, 2);
        for (int i = 0; i < 2; ++i) {
#if defined(ARDUINO)
            Serial.printf("CMD%d: 0x%02x", i + 1, buffer[i]);
#else
            printf("CTRL%d: 0x%02x", i + 1, buffer[i]);
#endif
#if defined(ARDUINO)
            Serial.print("\t\t BIN:");
            Serial.println(buffer[i], BIN);
#else
            LOG("\n");
#endif
        }
#if defined(ARDUINO)
        Serial.println();
#else
        printf("\n");
#endif
    }

private:

    float _convertAngleToPositive(float angle)
    {
        if (angle >= 360.0) {
            angle = angle - 360.0;
        }
        if (angle < 0) {
            angle = angle + 360.0;
        }
        return angle;
    }


    bool initImpl()
    {
        reset();
        return getChipID() == QMC6310_DEFAULT_ID;
    }

    int getReadMaskImpl()
    {
        return -1;
    }

protected:
    int16_t raw[3];
    float mag[3];
    float _declination;
    float sensitivity;
    int16_t x_offset = 0, y_offset = 0, z_offset = 0;
};

