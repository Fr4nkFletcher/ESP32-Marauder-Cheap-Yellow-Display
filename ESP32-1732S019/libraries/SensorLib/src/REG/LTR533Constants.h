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
 * @file      LTR533Constants.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-09
 */
#pragma once

#define LTR553_SLAVE_ADDRESS            (0x23)

#define LTR553_REG_ALS_CONTR            (0x80)
#define LTR553_REG_PS_CONTR             (0x81)
#define LTR553_REG_PS_LED               (0x82)
#define LTR553_REG_PS_N_PULSES          (0x83)
#define LTR553_REG_PS_MEAS_RATE         (0x84)
#define LTR553_REG_ALS_MEAS_RATE        (0x85)
#define LTR553_REG_PART_ID              (0x86)
#define LTR553_REG_MANUFAC_ID           (0x87)
#define LTR553_REG_ALS_DATA_CH1_0       (0x88)
#define LTR553_REG_ALS_DATA_CH1_1       (0x89)
#define LTR553_REG_ALS_DATA_CH0_0       (0x8A)
#define LTR553_REG_ALS_DATA_CH0_1       (0x8B)
#define LTR553_REG_ALS_PS_STATUS        (0x8C)
#define LTR553_REG_PS_DATA_0            (0x8D)
#define LTR553_REG_PS_DATA_1            (0x8E)
#define LTR553_REG_INTERRUPT            (0x8F)
#define LTR553_REG_PS_THRES_UP_0        (0x90)
#define LTR553_REG_PS_THRES_UP_1        (0x91)
#define LTR553_REG_PS_THRES_LOW_0       (0x92)
#define LTR553_REG_PS_THRES_LOW_1       (0x93)
#define LTR553_REG_PS_OFFSET_1          (0x94)
#define LTR553_REG_PS_OFFSET_0          (0x95)
#define LTR553_REG_ALS_THRES_UP_0       (0x97)
#define LTR553_REG_ALS_THRES_UP_1       (0x98)
#define LTR553_REG_ALS_THRES_LOW_0      (0x99)
#define LTR553_REG_ALS_THRES_LOW_1      (0x9A)
#define LTR553_REG_INTERRUPT_PERSIST    (0x9E)



#define LTR553_DEFAULT_MAN_ID           (0x05)