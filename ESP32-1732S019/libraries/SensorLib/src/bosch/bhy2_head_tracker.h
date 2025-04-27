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
* @file       bhy2_head_tracker.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef _BHY2_HEAD_TRACKER_H_
#define _BHY2_HEAD_TRACKER_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>

#include "bhy2.h"
#include "bhy2_head_tracker_defs.h"

/*!
 * @brief Parsing callback for Head  Orientation Quaternion
 *
 * @param[in] payload
 * @param[out] bhy2_head_tracker_quat_data to store Head Orientation [Quaternion] data
 *
 * @return  API error codes
 *
 */
void bhy2_head_tracker_quat_parsing(const uint8_t *payload, struct bhy2_head_tracker_quat_data *data);

/*!
 * @brief Parsing callback for Head  Orientation Euler
 *
 * @param[in] payload
 * @param[out] bhy2_head_tracker_eul_data to store Head Orientation [Euler] data
 *
 * @return  API error codes
 *
 */
void bhy2_head_tracker_eul_parsing(const uint8_t *payload, struct bhy2_head_tracker_eul_data *data);

/*!
 * @brief To trigger the Head Misalignment Calibration
 *
 * @param[in] buffer
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_trigger_hmc_calibration(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Calibration Configuration
 *
 * @param[in] buffer to hold the Head Misalignment Calibration Configuration
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_set_hmc_configuration(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Calibration Configuration
 *
 * @param[out] buffer to hold the Head Misalignment Calibration Configuration
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_get_hmc_configuration(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To set the default Head Misalignment Calibration Configuration
 *
 * @param[in] command buffer
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_set_default_hmc_configuration(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Calibrator Version
 *
 * @param[out] buffer to hold the Head Misalignment Calibrator Version
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_get_hmc_version(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[in] buffer to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_set_hmc_quat_calib_corr_config(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[out] buffer to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_get_hmc_quat_calib_corr_config(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[in] buffer to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_set_quat_initial_head_correction(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[out] buffer to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_get_quat_initial_head_correction(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the IMU/NDOF Head Orientation Version
 *
 * @param[out] buffer to hold the IMU/NDOF Head Orientation Version
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_get_ho_version(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Euler Initial Head Correction
 *
 * @param[in] buffer to hold the Head Misalignment Euler Initial Head Correction
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_set_eul_initial_head_correction(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Euler Initial Head Correction
 *
 * @param[out] buffer to hold the Head Misalignment Euler Initial Head Correction
 * @param[in] dev hub handle
 *
 * @return  API error codes
 *
 */
int8_t bhy2_head_tracker_get_eul_initial_head_correction(const void *buffer, struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHY2_HEAD_TRACKER_H_ */
