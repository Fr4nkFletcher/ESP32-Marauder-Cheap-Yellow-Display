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
 * @file      CM32181Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-14
 *
 */

#pragma once

#define CM32181_SLAVE_ADDRESS       (0x10)



// CM32181 slave address can be 0x10 or 0x48, determined by pin ADDR configuration
#define CM32181_ADDR_ALS            (0x10) // 7-bit
#define CM32181_ADDR_ALS1           (0x48) // 7-bit

// CM32181 registers
#define CM32181_REG_ALS_CONF        (uint8_t)(0x00)
#define CM32181_REG_ALS_THDH        (uint8_t)(0x01)
#define CM32181_REG_ALS_THDL        (uint8_t)(0x02)
#define CM32181_REG_ALS_PSM         (uint8_t)(0x03)
#define CM32181_REG_ALS_DATA        (uint8_t)(0x04)
#define CM32181_REG_ALS_STATUS      (uint8_t)(0x06)
#define CM32181_REG_ID              (uint8_t)(0x07)

#define CM32181_CHIP_ID             (0x81)












