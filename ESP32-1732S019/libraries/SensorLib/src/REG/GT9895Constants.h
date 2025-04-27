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
 * @file      GT9895Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */

#pragma once

#define GT9895_SLAVE_ADDRESS_H              (0x14)
#define GT9895_SLAVE_ADDRESS_L              (0x5D)

#define GT9895_MAX_TOUCH                    (10)
#define GT9895_MAX_PEN_KEY			        (2)
#define GT9895_INFO_MAX_LENGTH              (1024)
#define GT9895_MAX_SCAN_RATE_NUM            (8)
#define GT9895_MAX_SCAN_FREQ_NUM            (8)
#define GT9895_MAX_FREQ_NUM_STYLUS          (8)
#define GT9895_GESTURE_DATA_LEN             (16)
#define GT9895_IRQ_EVENT_HEAD_LEN           (8)
#define GT9895_BYTES_PER_POINT              (8)
#define GT9895_COOR_DATA_CHECKSUM_SIZE      (2)

#define GT9895_POINT_TYPE_STYLUS_HOVER      (0x01)
#define GT9895_POINT_TYPE_STYLUS            (0x03)


#define GT9895_TOUCH_EVENT                  (0x80)
#define GT9895_REQUEST_EVENT                (0x40)
#define GT9895_GESTURE_EVENT                (0x20)
#define GT9895_FP_EVENT                     (0x08)

#define GT9895_REG_FW_VERSION               (0x00010014u)
#define GT9895_REG_INFO                     (0x00010070u)
#define GT9895_REG_CMD                      (0x00010174u)
#define GT9895_REG_POINT                    (0x00010308u)


