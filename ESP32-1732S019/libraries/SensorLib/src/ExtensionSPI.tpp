/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      ExtensionSPI.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-01-19
 *
 */
#pragma once

#include "SensorLib.h"

template <class chipType>
class ExtensionSPI
{
public:
    void beginSPI(int mosi,
                  int miso,
                  int sck,
                  int cs
                 )
    {
        _mosi = mosi;
        _miso = miso;
        _sck = sck;
        _cs = cs;

        thisChip().pinMode(_mosi, OUTPUT);
        thisChip().pinMode(_sck, OUTPUT);

        if (_cs != SENSOR_PIN_NONE) {
            thisChip().pinMode(_cs, OUTPUT);
            thisChip().digitalWrite(_cs, HIGH);
        }

        if (_miso != SENSOR_PIN_NONE) {
            thisChip().pinMode(_miso, INPUT);
        }
        thisChip().digitalWrite(_mosi, HIGH);
        thisChip().digitalWrite(_sck, HIGH);

    }

    void setBitOrder(uint8_t order)
    {
        _order = order & 1;
    }

    uint8_t transfer8(uint8_t val)
    {
        return transferDataBits(val, 8);
    }

    uint16_t transfer9(uint16_t val)
    {
        return transferDataBits(val, 9);
    }

    uint32_t transferDataBits(uint32_t val, uint32_t bits)
    {
        uint32_t mask = _BV(bits - 1);
        uint8_t out = 0;
        if (_cs != SENSOR_PIN_NONE) {
            thisChip().digitalWrite(_cs, LOW);
        }
        for (int i = 0; i < bits ; i++) {
            thisChip().digitalWrite(_sck, LOW);
            out <<= 1;
            if (_miso != SENSOR_PIN_NONE) {
                if (thisChip().digitalRead(_miso)) {
                    out |= 0x01;
                }
            }
            thisChip().digitalWrite(_mosi, val & mask ?  HIGH : LOW);
            val <<= 1;
            thisChip().digitalWrite(_sck, HIGH);
        }
        if (_cs != SENSOR_PIN_NONE) {
            thisChip().digitalWrite(_cs, HIGH);
        }
        return out;
    }


protected:

    inline const chipType &thisChip() const
    {
        return static_cast<const chipType &>(*this);
    }

    inline chipType &thisChip()
    {
        return static_cast<chipType &>(*this);
    }

private:
    int _mosi;
    int _sck;
    int _miso;
    int _order;
    int _cs;
};