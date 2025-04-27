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
 * @file      MPU6886Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */

#pragma once


#define MPU6886_SLAVE_ADDRESS               (0x68)

#define MPU6886_REG_XG_OFFS_TC_H            (0x04)
#define MPU6886_REG_XG_OFFS_TC_L            (0x05)
#define MPU6886_REG_YG_OFFS_TC_H            (0x07)
#define MPU6886_REG_YG_OFFS_TC_L            (0x08)
#define MPU6886_REG_ZG_OFFS_TC_H            (0x0A)
#define MPU6886_REG_ZG_OFFS_TC_L            (0x0B)
#define MPU6886_REG_SELF_TEST_X_ACCEL       (0x0D)
#define MPU6886_REG_SELF_TEST_Y_ACCEL       (0x0E)
#define MPU6886_REG_SELF_TEST_Z_ACCEL       (0x0F)
#define MPU6886_REG_XG_OFFS_USRH            (0x13)
#define MPU6886_REG_XG_OFFS_USRL            (0x14)
#define MPU6886_REG_YG_OFFS_USRH            (0x15)
#define MPU6886_REG_YG_OFFS_USRL            (0x16)
#define MPU6886_REG_ZG_OFFS_USRH            (0x17)
#define MPU6886_REG_ZG_OFFS_USRL            (0x18)
#define MPU6886_REG_SMPLRT_DIV              (0x19)
#define MPU6886_REG_CONFIG                  (0x1A)
#define MPU6886_REG_GYRO_CONFIG             (0x1B)
#define MPU6886_REG_ACCEL_CONFIG            (0x1C)
#define MPU6886_REG_ACCEL_CONFIG_2          (0x1D)
#define MPU6886_REG_LP_MODE_CFG             (0x1E)
#define MPU6886_REG_ACCEL_WOM_X_THR         (0x20)
#define MPU6886_REG_ACCEL_WOM_Y_THR         (0x21)
#define MPU6886_REG_ACCEL_WOM_Z_THR         (0x22)
#define MPU6886_REG_FIFO_EN                 (0x23)
#define MPU6886_REG_FSYNC_INT               (0x36)
#define MPU6886_REG_INT_PIN_CFG             (0x37)
#define MPU6886_REG_INT_ENABLE              (0x38)
#define MPU6886_REG_FIFO_WM_INT_STATUS      (0x39)
#define MPU6886_REG_INT_STATUS              (0x3A)
#define MPU6886_REG_ACCEL_XOUT_H            (0x3B)
#define MPU6886_REG_ACCEL_XOUT_L            (0x3C)
#define MPU6886_REG_ACCEL_YOUT_H            (0x3D)
#define MPU6886_REG_ACCEL_YOUT_L            (0x3E)
#define MPU6886_REG_ACCEL_ZOUT_H            (0x3F)
#define MPU6886_REG_ACCEL_ZOUT_L            (0x40)
#define MPU6886_REG_TEMP_OUT_H              (0x41)
#define MPU6886_REG_TEMP_OUT_L              (0x42)
#define MPU6886_REG_GYRO_XOUT_H             (0x43)
#define MPU6886_REG_GYRO_XOUT_L             (0x44)
#define MPU6886_REG_GYRO_YOUT_H             (0x45)
#define MPU6886_REG_GYRO_YOUT_L             (0x46)
#define MPU6886_REG_GYRO_ZOUT_H             (0x47)
#define MPU6886_REG_GYRO_ZOUT_L             (0x48)
#define MPU6886_REG_SELF_TEST_X_GYRO        (0x50)
#define MPU6886_REG_SELF_TEST_Y_GYRO        (0x51)
#define MPU6886_REG_SELF_TEST_Z_GYRO        (0x52)
#define MPU6886_REG_E_ID0                   (0x53)
#define MPU6886_REG_E_ID1                   (0x54)
#define MPU6886_REG_E_ID2                   (0x55)
#define MPU6886_REG_E_ID3                   (0x56)
#define MPU6886_REG_E_ID4                   (0x57)
#define MPU6886_REG_E_ID5                   (0x58)
#define MPU6886_REG_E_ID6                   (0x59)
#define MPU6886_REG_FIFO_WM_TH1             (0x60)
#define MPU6886_REG_FIFO_WM_TH2             (0x61)
#define MPU6886_REG_SIGNAL_PATH_RESET       (0x68)
#define MPU6886_REG_ACCEL_INTEL_CTRL        (0x69)
#define MPU6886_REG_USER_CTRL               (0x6A)
#define MPU6886_REG_PWR_MGMT_1              (0x6B)
#define MPU6886_REG_PWR_MGMT_2              (0x6C)
#define MPU6886_REG_I2C_IF                  (0x70)
#define MPU6886_REG_FIFO_COUNTH             (0x72)
#define MPU6886_REG_FIFO_COUNTL             (0x73)
#define MPU6886_REG_FIFO_R_W                (0x74)
#define MPU6886_REG_WHO_AM_I                (0x75)
#define MPU6886_REG_XA_OFFSET_H             (0x77)
#define MPU6886_REG_XA_OFFSET_L             (0x78)
#define MPU6886_REG_YA_OFFSET_H             (0x7A)
#define MPU6886_REG_YA_OFFSET_L             (0x7B)
#define MPU6886_REG_ZA_OFFSET_H             (0x7D)
#define MPU6886_REG_ZA_OFFSET_L             (0x7E)


#define MPU6886_WHO_AM_I_RES                (0x19)





