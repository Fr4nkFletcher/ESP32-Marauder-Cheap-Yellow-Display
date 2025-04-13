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
 * @file      SensorWireHelper.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-05
 *
 */
#include "SensorWireHelper.h"

#if defined(ARDUINO)



int SensorWireHelper::regdump(TwoWire &w, Stream &serial, uint8_t devAddr, uint8_t start, uint8_t length)
{
    uint8_t *buffer =  (uint8_t *)malloc(length);
    if (!buffer) return -1;
    w.begin();
    w.beginTransmission(devAddr);
    w.write(start);
    if (w.endTransmission() != 0) {
        free(buffer);
        return -1;
    }
    size_t len = w.requestFrom(devAddr, length);
    if (length != len) {
        free(buffer);
        return -1;
    }
    w.readBytes(buffer, length);
    hexdump(buffer, length, serial);
    free(buffer);
    return 0;
}



void SensorWireHelper::dumpDevices(TwoWire &w, Stream &serial, int first, int last)
{
    int i, j;
    int ret;
    serial.printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (i = 0; i < 128; i += 16) {
        serial.printf("%02x: ", i);

        for (j = 0; j < 16; j++) {
            /* Skip unwanted addresses */
            if (i + j < first || i + j > last) {
                serial.printf("   ");
                continue;
            }

            w.beginTransmission(i + j);
            ret = w.endTransmission();

            if (ret != 0)
                serial.printf("-- ");
            else
                serial.printf("%02x ", i + j);
        }
        serial.printf("\n");
    }
}


void SensorWireHelper::hexdump(uint8_t *data, size_t len, Stream &serial)
{
    size_t rem_len = len;
    for (size_t i = 0; i < len; i += 16) {
        char str[80];
        snprintf(str, 80, "%07x  ", i);
        size_t line_len = 16;
        if (rem_len < line_len) {
            line_len = rem_len;
        }
        for (size_t j = 0; j < line_len; j++) {
            snprintf(&str[8 + j * 3], 80, "%02x ", data[i + j]);
        }
        for (size_t j = line_len; j < 16; j++) {
            snprintf(&str[8 + j * 3], 80, "   ");
        }
        str[56] = '|';
        str[57] = ' ';
        for (size_t j = 0; j < line_len; j++) {
            char c = data[i + j];
            if ((c < ' ') || (c > '~')) {
                c = '.';
            }
            snprintf(&str[58 + j], 80, "%c", c);
        }
        for (size_t j = line_len; j < 16; j++) {
            snprintf(&str[58 + j], 80, "   ");
        }
        serial.println(str);
        rem_len -= 16;
    }
}



#endif //ARDUINO
