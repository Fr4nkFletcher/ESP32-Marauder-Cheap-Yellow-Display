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
 * @file       bhi3.c
 * @date       2023-03-24
 * @version    v1.6.0
 *
 */

/*********************************************************************/
/* system header files */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*********************************************************************/
/* BHI3 SensorAPI header files */
#include "bhi3.h"

/*************************Private Functions***************************/

/*!
 * @brief To set the Physical Sensor Control Parameters
 */
int8_t bhi3_physical_sensor_control_set_config(uint8_t sensor_id,
                                               const uint8_t *payload,
                                               uint16_t len,
                                               uint8_t control_code,
                                               struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    uint16_t cmnd_len = BHY2_LE24MUL(len + 1); /*1 byte added for control code */
    uint8_t cmnd[cmnd_len];

    memset(cmnd, 0, cmnd_len);
    cmnd[0] = BHI3_PHY_SENSOR_CTRL_CODE(BHI3_PHY_SENSOR_CTRL_WRITE, control_code);

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        if (payload == NULL)
        {
            rslt = BHY2_E_NULL_PTR;
        }
        else
        {
            for (int i = 0; i < len; i++)
            {
                cmnd[i + 1] = *(payload + i);
            }

            rslt = bhy2_set_parameter(BHI3_PHY_SENSOR_CTRL_PARAM(sensor_id), cmnd, cmnd_len, dev);
        }
    }

    return rslt;
}

/*!
 * @brief To get the Physical Sensor Control Parameters
 */
int8_t bhi3_physical_sensor_control_get_config(uint8_t sensor_id,
                                               uint8_t *payload,
                                               uint32_t len,
                                               uint8_t control_code,
                                               struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t ret_len = 0;

    uint8_t cmnd[BHY2_COMMAND_HEADER_LEN] = {0};
    uint8_t rdbck[BHY2_LE24MUL(len)];

    cmnd[0] = BHI3_PHY_SENSOR_CTRL_CODE(BHI3_PHY_SENSOR_CTRL_READ, control_code);

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_set_parameter(BHI3_PHY_SENSOR_CTRL_PARAM(sensor_id), cmnd, BHY2_COMMAND_HEADER_LEN, dev);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        rslt = bhy2_get_parameter(BHI3_PHY_SENSOR_CTRL_PARAM(sensor_id), rdbck, BHY2_LE24MUL(len), &ret_len, dev);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        memcpy(payload, &rdbck[1], len); /*0th element holds control code */
    }

    return rslt;
}

/**************************Public Functions***************************/

/*!
 * @brief To parse Wrist Gesture Detection Data
 */
int8_t bhi3_wrist_gesture_detect_parse_data(const uint8_t *data, bhi3_wrist_gesture_detect_t *output)
{
    int8_t rslt = BHY2_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        output->wrist_gesture = (enum bhi3_wrist_gesture_activity) * (data);
    }

    return rslt;
}

/*!
 * @brief To set the Accelerometer Fast Offset Calibration
 */
int8_t bhi3_set_acc_foc(const struct bhy2_data_xyz *acc_foc, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_ACCELEROMETER,
                                                       (uint8_t *)acc_foc,
                                                       BHI3_PHY_ACC_FOC_CTRL_LEN,
                                                       BHI3_PHY_ACC_FOC_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Accelerometer Fast Offset Calibration
 */
int8_t bhi3_get_acc_foc(struct bhy2_data_xyz *acc_foc, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_ACCELEROMETER,
                                                       (uint8_t *)acc_foc,
                                                       BHI3_PHY_ACC_FOC_CTRL_LEN,
                                                       BHI3_PHY_ACC_FOC_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Accelerometer Power Mode
 */
int8_t bhi3_set_acc_power_mode(const uint8_t *acc_pwm, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_ACCELEROMETER,
                                                       acc_pwm,
                                                       BHI3_PHY_ACC_LOW_POWER_MODE_CTRL_LEN,
                                                       BHI3_PHY_ACC_LOW_POWER_MODE_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Accelerometer Power Mode
 */
int8_t bhi3_get_acc_power_mode(uint8_t *acc_pwm, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_ACCELEROMETER,
                                                       acc_pwm,
                                                       BHI3_PHY_ACC_LOW_POWER_MODE_CTRL_LEN,
                                                       BHI3_PHY_ACC_LOW_POWER_MODE_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Gyroscope Fast Offset Calibration
 */
int8_t bhi3_set_gyro_foc(const struct bhy2_data_xyz *gyro_foc, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       (uint8_t *)gyro_foc,
                                                       BHI3_PHY_GYRO_FOC_CTRL_LEN,
                                                       BHI3_PHY_GYRO_FOC_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Gyroscope Fast Offset Calibration
 */
int8_t bhi3_get_gyro_foc(struct bhy2_data_xyz *gyro_foc, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       (uint8_t *)gyro_foc,
                                                       BHI3_PHY_GYRO_FOC_CTRL_LEN,
                                                       BHI3_PHY_GYRO_FOC_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Gyroscope OIS
 */
int8_t bhi3_set_gyro_ois(const uint8_t *gyro_ois, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_ois,
                                                       BHI3_PHY_GYRO_OIS_CTRL_LEN,
                                                       BHI3_PHY_GYRO_OIS_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Gyroscope OIS
 */
int8_t bhi3_get_gyro_ois(uint8_t *gyro_ois, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_ois,
                                                       BHI3_PHY_GYRO_OIS_CTRL_LEN,
                                                       BHI3_PHY_GYRO_OIS_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Gyroscope Fast Startup
 */
int8_t bhi3_set_gyro_fast_startup(const uint8_t *gyro_fs, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_fs,
                                                       BHI3_PHY_GYRO_FAST_STARTUP_CTRL_LEN,
                                                       BHI3_PHY_GYRO_FAST_STARTUP_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Gyroscope Fast Startup
 */
int8_t bhi3_get_gyro_fast_startup(uint8_t *gyro_fs, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_fs,
                                                       BHI3_PHY_GYRO_FAST_STARTUP_CTRL_LEN,
                                                       BHI3_PHY_GYRO_FAST_STARTUP_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Gyroscope CRT
 */
int8_t bhi3_set_gyro_crt(const uint8_t *gyro_crt, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_crt,
                                                       BHI3_PHY_GYRO_CRT_CTRL_LEN,
                                                       BHI3_PHY_GYRO_CRT_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Gyroscope CRT
 */

/*lint -e506, -e778 */
int8_t bhi3_get_gyro_crt(uint8_t *gyro_crt, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt =
            bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                    gyro_crt,
                                                    BHY2_LE24MUL(BHI3_PHY_GYRO_CRT_CTRL_LEN),
                                                    BHI3_PHY_GYRO_CRT_CTRL_CODE,
                                                    dev);
    }

    return rslt;
}

/*!
 * @brief To set the Gyroscope Power Mode
 */
int8_t bhi3_set_gyro_power_mode(const uint8_t *gyro_pwm, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_pwm,
                                                       BHI3_PHY_GYRO_LOW_POWER_MODE_CTRL_LEN,
                                                       BHI3_PHY_GYRO_LOW_POWER_MODE_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Gyroscope Power Mode
 */
int8_t bhi3_get_gyro_power_mode(uint8_t *gyro_pwm, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_pwm,
                                                       BHI3_PHY_GYRO_LOW_POWER_MODE_CTRL_LEN,
                                                       BHI3_PHY_GYRO_LOW_POWER_MODE_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Gyroscope Timer Auto Trim state
 */
int8_t bhi3_set_gyro_timer_auto_trim(const uint8_t *gyro_tat, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_tat,
                                                       BHI3_PHY_GYRO_TIMER_AUTO_TRIM_CTRL_LEN,
                                                       BHI3_PHY_GYRO_TIMER_AUTO_TRIM_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Gyroscope Timer Auto Trim Status
 */
int8_t bhi3_get_gyro_timer_auto_trim(uint8_t *gyro_tat, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                                       gyro_tat,
                                                       BHI3_PHY_GYRO_TIMER_AUTO_TRIM_CTRL_LEN,
                                                       BHI3_PHY_GYRO_TIMER_AUTO_TRIM_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Wrist Wear Wakeup Configuration
 */
int8_t bhi3_set_wrist_wear_wakeup_config(const uint8_t *www_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP,
                                                       www_cnfg,
                                                       BHI3_PHY_WRIST_WEAR_WAKEUP_CTRL_LEN,
                                                       BHI3_PHY_WRIST_WEAR_WAKEUP_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Wrist Wear Wakeup Configuration
 */
int8_t bhi3_get_wrist_wear_wakeup_config(uint8_t *www_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP,
                                                       www_cnfg,
                                                       BHI3_PHY_WRIST_WEAR_WAKEUP_CTRL_LEN,
                                                       BHI3_PHY_WRIST_WEAR_WAKEUP_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Any Motion Configuration
 */
int8_t bhi3_set_anymotion_config(const uint8_t *any_motion_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_PHYS_ANY_MOTION,
                                                       any_motion_cnfg,
                                                       BHI3_PHY_ANY_MOTION_CTRL_LEN,
                                                       BHI3_PHY_ANY_MOTION_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Any Motion Configuration
 */
int8_t bhi3_get_anymotion_config(uint8_t *any_motion_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_PHYS_ANY_MOTION,
                                                       any_motion_cnfg,
                                                       BHI3_PHY_ANY_MOTION_CTRL_LEN,
                                                       BHI3_PHY_ANY_MOTION_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the No Motion Configuration
 */
int8_t bhi3_set_nomotion_config(const uint8_t *no_motion_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_PHYS_NO_MOTION,
                                                       no_motion_cnfg,
                                                       BHI3_PHY_NO_MOTION_CTRL_LEN,
                                                       BHI3_PHY_NO_MOTION_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the No Motion Configuration
 */
int8_t bhi3_get_nomotion_config(uint8_t *no_motion_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the configuration parameter */
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_PHYS_NO_MOTION,
                                                       no_motion_cnfg,
                                                       BHI3_PHY_NO_MOTION_CTRL_LEN,
                                                       BHI3_PHY_NO_MOTION_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To set the Wrist Gesture Detect Configuration
 */
int8_t bhi3_set_wrist_gesture_detect_config(const uint8_t *wgd_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the configuration parameter */
        rslt = bhi3_physical_sensor_control_set_config(BHY2_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT,
                                                       wgd_cnfg,
                                                       BHI3_PHY_WRIST_GESTURE_DETECT_CTRL_LEN,
                                                       BHI3_PHY_WRIST_GESTURE_DETECT_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}

/*!
 * @brief To get the Wrist Gesture Detect Configuration
 */
int8_t bhi3_get_wrist_gesture_detect_config(uint8_t *wgd_cnfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhi3_physical_sensor_control_get_config(BHY2_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT,
                                                       wgd_cnfg,
                                                       BHI3_PHY_WRIST_GESTURE_DETECT_CTRL_LEN,
                                                       BHI3_PHY_WRIST_GESTURE_DETECT_CTRL_CODE,
                                                       dev);
    }

    return rslt;
}
