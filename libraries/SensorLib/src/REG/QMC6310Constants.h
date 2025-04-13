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
 * @file      QMC6310Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */

#pragma once

// @brief  device address
#define QMC6310_SLAVE_ADDRESS                   (0x1C)
#define QMC6310_DEFAULT_ID                      (0x80)


#define QMC6310_REG_CHIP_ID                     (0x00)
#define QMC6310_REG_LSB_DX                      (0X01)
#define QMC6310_REG_MSB_DX                      (0X02)
#define QMC6310_REG_LSB_DY                      (0X03)
#define QMC6310_REG_MSB_DY                      (0X04)
#define QMC6310_REG_LSB_DZ                      (0X05)
#define QMC6310_REG_MSB_DZ                      (0X06)
#define QMC6310_REG_STAT                        (0X09)
#define QMC6310_REG_CMD1                        (0x0A)
#define QMC6310_REG_CMD2                        (0x0B)


#define QMC6310_REG_SIGN                        (0x29)



