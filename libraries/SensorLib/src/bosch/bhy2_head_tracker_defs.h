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
* @file       bhy2_head_tracker_defs.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef _BHY2_HEAD_TRACKER_DEFS_H_
#define _BHY2_HEAD_TRACKER_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>

#include "bhy2.h"

/*! Virtual Sensor Macros */
#define BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG               UINT8_C(120)      /*Head Orientation Misalignment*/
#define BHY2_SENSOR_ID_IMU_HEAD_ORI_Q                 UINT8_C(121)      /*IMU Head Orientation Quaternion*/
#define BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q                UINT8_C(122)      /*NDOF Head Orientation Quaternion*/
#define BHY2_SENSOR_ID_IMU_HEAD_ORI_E                 UINT8_C(123)      /*IMU Head Orientation Euler*/
#define BHY2_SENSOR_ID_NDOF_HEAD_ORI_E                UINT8_C(124)      /*NDOF Head Orientation Euler*/

#define BHY2_HEAD_ORI_PAGE                            UINT16_C(12)
#define BHY2_HEAD_ORI_PARAM(id)                       (((BHY2_HEAD_ORI_PAGE) << 8) | (id))

#define BHY2_HEAD_ORI_HMC_TRIGGER_CALIB               UINT8_C(1)
#define BHY2_HEAD_ORI_HMC_TRIGGER_CALIB_LENGTH        UINT8_C(4)
#define BHY2_HEAD_ORI_HMC_TRIGGER_CALIB_SET           UINT8_C(1)

#define BHY2_HEAD_ORI_HMC_CONFIG                      UINT8_C(2)
#define BHY2_HEAD_ORI_HMC_CONFIG_LENGTH               UINT8_C(8)

#define BHY2_HEAD_ORI_HMC_SET_DEF_CONFIG              UINT8_C(3)
#define BHY2_HEAD_ORI_HMC_SET_DEF_CONFIG_LENGTH       UINT8_C(4)
#define BHY2_HEAD_ORI_HMC_SET_DEF_CONFIG_SET          UINT8_C(1)

#define BHY2_HEAD_ORI_HMC_VERSION                     UINT8_C(4)
#define BHY2_HEAD_ORI_HMC_VERSION_LENGTH              UINT8_C(4)

#define BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR             UINT8_C(5)
#define BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR_WLENGTH     UINT8_C(16)
#define BHY2_HEAD_ORI_HMC_QUAT_CALIB_CORR_RLENGTH     UINT8_C(20)

#define BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR          UINT8_C(10)
#define BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_LENGTH   UINT8_C(4)
#define BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_DISABLE  UINT8_C(0)
#define BHY2_HEAD_ORI_QUAT_INITIAL_HEAD_CORR_ENABLE   UINT8_C(1)

#define BHY2_HEAD_ORI_VERSION                         UINT8_C(11)
#define BHY2_HEAD_ORI_VERSION_LENGTH                  UINT8_C(4)

#define BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR           UINT8_C(12)
#define BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_LENGTH    UINT8_C(4)
#define BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_DISABLE   UINT8_C(0)
#define BHY2_HEAD_ORI_EUL_INITIAL_HEAD_CORR_ENABLE    UINT8_C(1)

/*! Sensor Structure for Head  Orientation Quaternion */
struct bhy2_head_tracker_quat_data
{
    int16_t x, y, z, w;
    uint8_t accuracy;
};

/*! Sensor Structure for Head  Orientation Euler */
struct bhy2_head_tracker_eul_data
{
    int16_t heading, pitch, roll;
    uint8_t accuracy;
};

/*! Structure for Head  Misalignment Configuration */
struct bhy2_head_tracker_misalignment_config
{
    uint8_t still_phase_max_dur;
    uint8_t still_phase_min_dur;
    uint8_t still_phase_max_samples;
    int32_t acc_diff_threshold;
};

/*! Structure for Head Orientation /Head  Misalignment version */
struct bhy2_head_tracker_ver
{
    uint8_t major, minor, patch, reserved;
};

/*! Structure for Head  Misalignment Quaternion Correction */
struct bhy2_head_tracker_misalignment_quat_corr
{
    int32_t quaternion_x;
    int32_t quaternion_y;
    int32_t quaternion_z;
    int32_t quaternion_w;
    int32_t accuracy;
};

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHY2_HEAD_TRACKER_DEFS_H_ */
