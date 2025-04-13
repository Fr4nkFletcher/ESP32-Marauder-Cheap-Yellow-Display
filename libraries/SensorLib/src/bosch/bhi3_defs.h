/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bhi3_defs.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHI3_DEFS_H__
#define __BHI3_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhy2_defs.h"

/*! The Product ID needs to be replaced with Chip ID for explicit HW specific Identification*/
#define BHI3_PRODUCT_ID                          BHY2_PRODUCT_ID

/*! Firmware header identifier */
/*#define BHY2_FW_MAGIC                            UINT16_C(0x662B) */

/*! BHI3 Variant IDs */
#define BHI3_VARIANT_ID_BHI360                   UINT32_C(0x18BC5434)
#define BHI3_VARIANT_ID_BHI380                   UINT32_C(0x6E6D6CA6)

/*! BHI3 Specifc Sensor IDs */
#define BHI3_SENSOR_ID_AR_WEAR_WU                UINT8_C(154) /* Activity Recognition (wear/hear) */
#define BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU   UINT8_C(156) /* Wrist Gesture Detector Low Power Wakeup*/
#define BHI3_SENSOR_ID_WRIST_WEAR_LP_WU          UINT8_C(158) /* Wrist Wear Low Power Wakeup */
#define BHI3_SENSOR_ID_NO_MOTION_LP_WU           UINT8_C(159) /* No motion Low Power */

/*! Physical Sensor Control Parameter Page Base Address*/
#define BHI3_PHY_SENSOR_CTRL_PAGE                UINT16_C(0x0E)

/*! Physical Sensor Control pages
 * Here the 'id' refers to the Sensor ID of requisite Physical Sensors (Acc/Gyro)
 * */
#define BHI3_PHY_SENSOR_CTRL_PARAM(id)           (((BHI3_PHY_SENSOR_CTRL_PAGE) << 8) | (id))
#define BHI3_PHY_SENSOR_CTRL_CODE(dir, code)     ((dir << 7) | (code))

#define BHI3_PHY_SENSOR_CTRL_WRITE               UINT8_C(0x00)
#define BHI3_PHY_SENSOR_CTRL_READ                UINT8_C(0x01)

#define BHI3_PHY_ACC_FOC_CTRL_CODE               UINT8_C(0x01)
#define BHI3_PHY_ACC_FOC_CTRL_LEN                6

#define BHI3_PHY_ACC_LOW_POWER_MODE_CTRL_CODE    UINT8_C(0x05)
#define BHI3_PHY_ACC_NORMAL_POWER_MODE           UINT8_C(0x00)
#define BHI3_PHY_ACC_LOW_POWER_MODE              UINT8_C(0x02)
#define BHI3_PHY_ACC_LOW_POWER_MODE_CTRL_LEN     1

#define BHI3_PHY_GYRO_FOC_CTRL_CODE              UINT8_C(0x01)
#define BHI3_PHY_GYRO_FOC_CTRL_LEN               6

#define BHI3_PHY_GYRO_OIS_CTRL_CODE              UINT8_C(0x02)
#define BHI3_PHY_GYRO_DISABLE_OIS                UINT8_C(0x00)
#define BHI3_PHY_GYRO_ENABLE_OIS                 UINT8_C(0x01)
#define BHI3_PHY_GYRO_OIS_CTRL_LEN               1

#define BHI3_PHY_GYRO_FAST_STARTUP_CTRL_CODE     UINT8_C(0x03)
#define BHI3_PHY_GYRO_DISABLE_FAST_STARTUP       UINT8_C(0x00)
#define BHI3_PHY_GYRO_ENABLE_FAST_STARTUP        UINT8_C(0x01)
#define BHI3_PHY_GYRO_FAST_STARTUP_CTRL_LEN      1

#define BHI3_PHY_GYRO_CRT_CTRL_CODE              UINT8_C(0x04)
#define BHI3_PHY_GYRO_CRT_STATUS_SUCCESS         UINT8_C(0x00)
#define BHI3_PHY_GYRO_ENABLE_CRT                 UINT8_C(0x01)
#define BHI3_PHY_GYRO_CRT_CTRL_LEN               1

#define BHI3_PHY_GYRO_LOW_POWER_MODE_CTRL_CODE   UINT8_C(0x05)
#define BHI3_PHY_GYRO_NORMAL_POWER_MODE          UINT8_C(0x00)
#define BHI3_PHY_GYRO_PERFORMANCE_POWER_MODE     UINT8_C(0x01)
#define BHI3_PHY_GYRO_LOW_POWER_MODE             UINT8_C(0x02)
#define BHI3_PHY_GYRO_LOW_POWER_MODE_CTRL_LEN    1

#define BHI3_PHY_GYRO_TIMER_AUTO_TRIM_CTRL_CODE  UINT8_C(0x06)
#define BHI3_PHY_GYRO_DISABLE_TIMER_AUTO_TRIM    UINT8_C(0x00)
#define BHI3_PHY_GYRO_ENABLE_TIMER_AUTO_TRIM     UINT8_C(0x01)
#define BHI3_PHY_GYRO_TIMER_AUTO_TRIM_CTRL_LEN   1

#define BHI3_PHY_WRIST_WEAR_WAKEUP_CTRL_CODE     UINT8_C(0x01)
#define BHI3_PHY_WRIST_WEAR_WAKEUP_CTRL_LEN      10

#define BHI3_PHY_ANY_MOTION_CTRL_CODE            UINT8_C(0x01)
#define BHI3_PHY_ANY_MOTION_CTRL_LEN             4

#define BHI3_PHY_NO_MOTION_CTRL_CODE             UINT8_C(0x01)
#define BHI3_PHY_NO_MOTION_CTRL_LEN              4

#define BHI3_PHY_WRIST_GESTURE_DETECT_CTRL_CODE  UINT8_C(0x07)
#define BHI3_PHY_WRIST_GESTURE_DETECT_CTRL_LEN   19

typedef struct bhi3_wrist_wear_wakeup_config_param {
    uint16_t min_angle_focus;
    uint16_t min_angle_nonfocus;
    uint8_t angle_landscape_right;
    uint8_t angle_landscape_left;
    uint8_t angle_portrait_down;
    uint8_t angle_portrait_up;
    uint8_t min_dur_moved;
    uint8_t min_dur_quite;
} __attribute__ ((packed)) bhi3_wrist_wear_wakeup_config_param_t;

typedef struct bhi3_any_no_motion_config_param {
    uint16_t duration;
    uint8_t axis;
    uint16_t threshold;
} __attribute__ ((packed)) bhi3_any_no_motion_config_param_t;

typedef struct bhi3_wrist_gesture_detect_config_param {
    uint16_t min_flick_peak_y_threshold;
    uint16_t min_flick_peak_z_threshold;
    uint16_t gravity_bounds_x_pos;
    uint16_t gravity_bounds_x_neg;
    uint16_t gravity_bounds_y_neg;
    uint16_t gravity_bounds_z_neg;
    uint16_t flick_peak_decay_coeff;
    uint16_t lp_mean_filter_coeff;
    uint16_t max_duration_jiggle_peaks;
    uint8_t device_position;
} __attribute__ ((packed)) bhi3_wrist_gesture_detect_config_param_t;

enum bhi3_wrist_gesture_activity {
    NO_GESTURE,
    WRIST_SHAKE_JIGGLE = 0x03,
    FLICK_IN,
    FLICK_OUT
};

typedef struct bhi3_wrist_gesture_detect {
    enum bhi3_wrist_gesture_activity wrist_gesture;
} __attribute__ ((packed)) bhi3_wrist_gesture_detect_t;

static const char *const bhi3_wrist_gesture_detect_output[] = {
    [NO_GESTURE] = "NO_GESTURE",
    [1] = "",
    [2] = "",
    [WRIST_SHAKE_JIGGLE] = "WRIST_SHAKE_JIGGLE",
    [FLICK_IN] = "FLICK_IN",
    [FLICK_OUT] = "FLICK_OUT"
}; /*lint -e528 */

enum bhi3_phy_sensor_acc_power_mode {
    ACC_NORMAL,
    ACC_LOW_POWER = 0x02
};

enum bhi3_phy_sensor_gyro_power_mode {
    GYRO_NORMAL,
    GYRO_PERFORMANCE,
    GYRO_LOW_POWER
};

static const char *const bhi3_phy_sensor_acc_pwm_output[] = {
    [ACC_NORMAL] = "NORMAL",
    [1] = "",
    [ACC_LOW_POWER] = "LOW POWER"
}; /*lint -e528 */

static const char *const bhi3_phy_sensor_gyro_pwm_output[] = {
    [GYRO_NORMAL] = "NORMAL",
    [GYRO_PERFORMANCE] = "PERFORMANCE",
    [GYRO_LOW_POWER] = "LOW POWER"
}; /*lint -e528 */

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI3_DEFS_H__ */
