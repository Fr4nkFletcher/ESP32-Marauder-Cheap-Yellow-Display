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
 * @file      GT911Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */

#pragma once

#define GT911_SLAVE_ADDRESS_H         (0x14)
#define GT911_SLAVE_ADDRESS_L         (0x5D)
#define GT911_DEV_ID                  (911)
#define GT911_BASE_REF_RATE           (5)
#define GT911_REG_LENGTH              (186)

// Real-time command (Write only)
#define GT911_COMMAND                 (0x8040)
#define GT911_ESD_CHECK               (0x8041)
#define GT911_COMMAND_CHECK           (0x8046)

// Configuration information (R/W)
#define GT911_CONFIG_START            (0x8047)
#define GT911_CONFIG_VERSION          (0x8047)
#define GT911_X_OUTPUT_MAX_LOW        (0x8048)
#define GT911_X_OUTPUT_MAX_HIGH       (0x8049)
#define GT911_Y_OUTPUT_MAX_LOW        (0x804A)
#define GT911_Y_OUTPUT_MAX_HIGH       (0x804B)
#define GT911_TOUCH_NUMBER            (0x804C)
#define GT911_MODULE_SWITCH_1         (0x804D)
#define GT911_MODULE_SWITCH_2         (0x804E)
#define GT911_SHAKE_COUNT             (0x804F)
#define GT911_FILTER                  (0x8050)
#define GT911_LARGE_TOUCH             (0x8051)
#define GT911_NOISE_REDUCTION         (0x8052)
#define GT911_SCREEN_TOUCH_LEVEL      (0x8053)
#define GT911_SCREEN_RELEASE_LEVEL    (0x8054)
#define GT911_LOW_POWER_CONTROL       (0x8055)
#define GT911_REFRESH_RATE            (0x8056)
#define GT911_X_THRESHOLD             (0x8057)
#define GT911_Y_THRESHOLD             (0x8058)
#define GT911_X_SPEED_LIMIT           (0x8059) // Reserve
#define GT911_Y_SPEED_LIMIT           (0x805A) // Reserve
#define GT911_SPACE_TOP_BOTTOM        (0x805B)
#define GT911_SPACE_LEFT_RIGHT        (0x805C)
#define GT911_MINI_FILTER             (0x805D)
#define GT911_STRETCH_R0              (0x805E)
#define GT911_STRETCH_R1              (0x805F)
#define GT911_STRETCH_R2              (0x8060)
#define GT911_STRETCH_RM              (0x8061)
#define GT911_DRV_GROUPA_NUM          (0x8062)
#define GT911_DRV_GROUPB_NUM          (0x8063)
#define GT911_SENSOR_NUM              (0x8064)
#define GT911_FREQ_A_FACTOR           (0x8065)
#define GT911_FREQ_B_FACTOR           (0x8066)
#define GT911_PANEL_BIT_FREQ_L        (0x8067)
#define GT911_PANEL_BIT_FREQ_H        (0x8068)
#define GT911_PANEL_SENSOR_TIME_L     (0x8069) // Reserve
#define GT911_PANEL_SENSOR_TIME_H     (0x806A)
#define GT911_PANEL_TX_GAIN           (0x806B)
#define GT911_PANEL_RX_GAIN           (0x806C)
#define GT911_PANEL_DUMP_SHIFT        (0x806D)
#define GT911_DRV_FRAME_CONTROL       (0x806E)
#define GT911_CHARGING_LEVEL_UP       (0x806F)
#define GT911_MODULE_SWITCH3          (0x8070)
#define GT911_GESTURE_DIS             (0X8071)
#define GT911_GESTURE_LONG_PRESS_TIME (0x8072)
#define GT911_X_Y_SLOPE_ADJUST        (0X8073)
#define GT911_GESTURE_CONTROL         (0X8074)
#define GT911_GESTURE_SWITCH1         (0X8075)
#define GT911_GESTURE_SWITCH2         (0X8076)
#define GT911_GESTURE_REFRESH_RATE    (0x8077)
#define GT911_GESTURE_TOUCH_LEVEL     (0x8078)
#define GT911_NEWGREENWAKEUPLEVEL     (0x8079)
#define GT911_FREQ_HOPPING_START      (0x807A)
#define GT911_FREQ_HOPPING_END        (0X807B)
#define GT911_NOISE_DETECT_TIMES      (0x807C)
#define GT911_HOPPING_FLAG            (0X807D)
#define GT911_HOPPING_THRESHOLD       (0X807E)
#define GT911_NOISE_THRESHOLD         (0X807F) // Reserve
#define GT911_NOISE_MIN_THRESHOLD     (0X8080)
#define GT911_HOPPING_SENSOR_GROUP    (0X8082)
#define GT911_HOPPING_SEG1_NORMALIZE  (0X8083)
#define GT911_HOPPING_SEG1_FACTOR     (0X8084)
#define GT911_MAIN_CLOCK_AJDUST       (0X8085)
#define GT911_HOPPING_SEG2_NORMALIZE  (0X8086)
#define GT911_HOPPING_SEG2_FACTOR     (0X8087)
#define GT911_HOPPING_SEG3_NORMALIZE  (0X8089)
#define GT911_HOPPING_SEG3_FACTOR     (0X808A)
#define GT911_HOPPING_SEG4_NORMALIZE  (0X808C)
#define GT911_HOPPING_SEG4_FACTOR     (0X808D)
#define GT911_HOPPING_SEG5_NORMALIZE  (0X808F)
#define GT911_HOPPING_SEG5_FACTOR     (0X8090)
#define GT911_HOPPING_SEG6_NORMALIZE  (0X8092)
#define GT911_KEY_1                   (0X8093)
#define GT911_KEY_2                   (0X8094)
#define GT911_KEY_3                   (0X8095)
#define GT911_KEY_4                   (0X8096)
#define GT911_KEY_AREA                (0X8097)
#define GT911_KEY_TOUCH_LEVEL         (0X8098)
#define GT911_KEY_LEAVE_LEVEL         (0X8099)
#define GT911_KEY_SENS_1_2            (0X809A)
#define GT911_KEY_SENS_3_4            (0X809B)
#define GT911_KEY_RESTRAIN            (0X809C)
#define GT911_KEY_RESTRAIN_TIME       (0X809D)
#define GT911_GESTURE_LARGE_TOUCH     (0X809E)
#define GT911_HOTKNOT_NOISE_MAP       (0X80A1)
#define GT911_LINK_THRESHOLD          (0X80A2)
#define GT911_PXY_THRESHOLD           (0X80A3)
#define GT911_GHOT_DUMP_SHIFT         (0X80A4)
#define GT911_GHOT_RX_GAIN            (0X80A5)
#define GT911_FREQ_GAIN0              (0X80A6)
#define GT911_FREQ_GAIN1              (0X80A7)
#define GT911_FREQ_GAIN2              (0X80A8)
#define GT911_FREQ_GAIN3              (0X80A9)
#define GT911_COMBINE_DIS             (0X80B3)
#define GT911_SPLIT_SET               (0X80B4)
#define GT911_SENSOR_CH0              (0X80B7)
#define GT911_DRIVER_CH0              (0X80D5)
#define GT911_CONFIG_CHKSUM           (0X80FF)
#define GT911_CONFIG_FRESH            (0X8100)
#define GT911_CONFIG_SIZE             (0xFF - 0x46)

// Coordinate information
#define GT911_PRODUCT_ID              (0x8140)
#define GT911_FIRMWARE_VERSION        (0x8144)
#define GT911_X_RESOLUTION            (0x8146)
#define GT911_Y_RESOLUTION            (0x8148)

#define GT911_VENDOR_ID               (0X8140)
#define GT911_IMFORMATION             (0X8140)
#define GT911_POINT_INFO              (0X814E)
#define GT911_POINT_1                 (0X814F)
#define GT911_POINT_2                 (0X8157)
#define GT911_POINT_3                 (0X815F)
#define GT911_POINT_4                 (0X8167)
#define GT911_POINT_5                 (0X816F)


#define GT911_GET_POINT(x)                  (x & 0x0F)
#define GT911_GET_BUFFER_STATUS(x)          (x & 0x80)
#define GT911_GET_HAVE_KEY(x)               (x & 0x10)


