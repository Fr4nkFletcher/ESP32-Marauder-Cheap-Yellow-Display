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
 * @file      FT6X36Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-01
 *
 */

#pragma once

#define FT3267_SLAVE_ADDRESS       (0x38)
#define FT5206_SLAVE_ADDRESS       (0x38)
#define FT6X36_SLAVE_ADDRESS       (0x38)

#define FT6X36_VEND_ID              (0x11)
#define FT3267_CHIP_ID              (0x33)
#define FT6206_CHIP_ID              (0x06)
#define FT6236_CHIP_ID              (0x36)
#define FT6236U_CHIP_ID             (0x64)
#define FT5206U_CHIP_ID             (0x64)

#define FT6X36_REG_MODE            (0x00)
#define FT6X36_REG_GEST            (0x01)
#define FT6X36_REG_STATUS          (0x02)
#define FT6X36_REG_TOUCH1_XH       (0x03)
#define FT6X36_REG_TOUCH1_XL       (0x04)
#define FT6X36_REG_TOUCH1_YH       (0x05)
#define FT6X36_REG_TOUCH1_YL       (0x06)
#define FT6X36_REG_THRESHOLD      (0x80)
#define FT6X36_REG_MONITOR_TIME     (0x87)
#define FT6X36_REG_PERIOD_ACTIVE    (0x88)
#define FT6X36_REG_PERIOD_MONITOR   (0x89)


#define FT6X36_REG_AUTO_CLB_MODE   (0xA0)
#define FT6X36_REG_LIB_VERSION_H      (0xA1)
#define FT6X36_REG_LIB_VERSION_L    (0xA2)
#define FT6X36_REG_INT_STATUS      (0xA4)
#define FT6X36_REG_POWER_MODE      (0xA5)
#define FT6X36_REG_FIRM_VERS        (0xA6)
#define FT6X36_REG_CHIP_ID          (0xA3)
#define FT6X36_REG_VENDOR1_ID      (0xA8)
#define FT6X36_REG_ERROR_STATUS    (0xA9)