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
* @file       bhy2_head_tracker.c
* @date       2023-03-24
* @version    v1.6.0
*
*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdlib.h>

/*********************************************************************/
/* BHY2 SensorAPI header files */
/*********************************************************************/
#include "bhy2.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhy2_head_tracker.h"

/*Parsing callback for Head  Orientation Quaternion*/
void bhy2_head_tracker_quat_parsing(const uint8_t *payload, struct bhy2_head_tracker_quat_data *data)
{
    uint8_t i = 0;

    if ((payload != NULL) && (data != NULL))
    {
        data->x = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->y = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->z = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->w = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->accuracy = payload[i];
    }
}

/*Parsing callback for Head  Orientation Euler*/
void bhy2_head_tracker_eul_parsing(const uint8_t *payload, struct bhy2_head_tracker_eul_data *data)
{
    uint8_t i = 0;

    if ((payload != NULL) && (data != NULL))
    {
        data->heading = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->pitch = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->roll = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->accuracy = payload[i];
    }
}

/*Callback for triggering Head Misalignment Calibration*/
int8_t bhy2_head_tracker_trigger_hmc_calibration(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Trigger Head Misalignment Calibration */
        rslt = bhy2_set_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_TRIGGER_CALIB),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_TRIGGER_CALIB_LENGTH,
                                  dev);
    }

    return rslt;
}

/*Callback for setting Head Misalignment Calibration Configuration*/
int8_t bhy2_head_tracker_set_hmc_configuration(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set Head Misalignment Configuration */
        rslt = bhy2_set_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_CONFIG),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_CONFIG_LENGTH,
                                  dev);
    }

    return rslt;
}

/*Callback for getting Head Misalignment Calibration Configuration*/
int8_t bhy2_head_tracker_get_hmc_configuration(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t act_len;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get Head Misalignment Configuration */
        rslt = bhy2_get_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_CONFIG),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_CONFIG_LENGTH,
                                  &act_len,
                                  dev);
    }

    return rslt;
}

/*Callback for setting default Head Misalignment Calibration Configuration*/
int8_t bhy2_head_tracker_set_default_hmc_configuration(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set default Head Misalignment Configuration */
        rslt = bhy2_set_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_SET_DEF_CONFIG),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_SET_DEF_CONFIG_LENGTH,
                                  dev);
    }

    return rslt;
}

/*Callback for getting Head Misalignment Calibrator Version*/
int8_t bhy2_head_tracker_get_hmc_version(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t act_len;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get Head Misalignment Calibrator Version */
        rslt = bhy2_get_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_VERSION),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_VERSION_LENGTH,
                                  &act_len,
                                  dev);
    }

    return rslt;
}

/*Callback for setting Head Misalignment Calibration Correction Quaternion Configuration*/
int8_t bhy2_head_tracker_set_hmc_quat_calib_corr_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set Head Misalignment Calibration Correction Quaternion Configuration */
        rslt = bhy2_set_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR_WLENGTH,
                                  dev);
    }

    return rslt;
}

/*Callback for getting Head Misalignment Calibration Correction Quaternion Configuration*/
int8_t bhy2_head_tracker_get_hmc_quat_calib_corr_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t act_len;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get Head Misalignment Calibration Correction Quaternion Configuration */
        rslt = bhy2_get_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR_RLENGTH,
                                  &act_len,
                                  dev);
    }

    return rslt;
}

/*Callback for setting Head Orientation Initial Head Correction for Quaternion*/
int8_t bhy2_head_tracker_set_quat_initial_head_correction(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set Head Orientation Initial Head Correction for Quaternion */
        rslt = bhy2_set_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_LENGTH,
                                  dev);
    }

    return rslt;
}

/*Callback for getting Head Orientation Initial Head Correction for Quaternion*/
int8_t bhy2_head_tracker_get_quat_initial_head_correction(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t act_len;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get Head Orientation Initial Head Correction for Quaternion */
        rslt = bhy2_get_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_LENGTH,
                                  &act_len,
                                  dev);
    }

    return rslt;
}

/*Callback for getting Head Orientation Version*/
int8_t bhy2_head_tracker_get_ho_version(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t act_len;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get IMU/NDOF Head Orientation Version */
        rslt = bhy2_get_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_VERSION),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_VERSION_LENGTH,
                                  &act_len,
                                  dev);
    }

    return rslt;
}

/*Callback for setting Head Orientation Initial Head Correction for Euler*/
int8_t bhy2_head_tracker_set_eul_initial_head_correction(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set Head Orientation Initial Head Correction for Euler */
        rslt = bhy2_set_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_LENGTH,
                                  dev);
    }

    return rslt;
}

/*Callback for getting Head Orientation Initial Head Correction for Euler*/
int8_t bhy2_head_tracker_get_eul_initial_head_correction(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t act_len;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get Head Orientation Initial Head Correction for Euler */
        rslt = bhy2_get_parameter(BHY2_HEAD_ORI_PARAM(BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR),
                                  (uint8_t*)buffer,
                                  BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_LENGTH,
                                  &act_len,
                                  dev);
    }

    return rslt;
}
