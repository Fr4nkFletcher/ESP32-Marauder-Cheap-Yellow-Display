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
* @file       bhi3.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHI3_H__
#define __BHI3_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhy2.h"
#include "bhi3_defs.h"

/*!
 * @brief To parse Wrist Gesture Detection Data
 *
 * @param[in] Wrist Gesture Detection Data
 * @param[in] Wrist Gesture Detection Output Structure
 *
 * @return API error codes
 *
 */
int8_t bhi3_wrist_gesture_detect_parse_data(const uint8_t *data, bhi3_wrist_gesture_detect_t *output);

/*!
 * @brief To set the Accelerometer Fast Offset Calibration
 *
 * @param[in] Physical Accelerometer FOC data
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_set_acc_foc(const struct bhy2_data_xyz *acc_foc, struct bhy2_dev *dev);

/*!
 * @brief To get the Accelerometer Fast Offset Calibration
 *
 * @param[out] Physical Accelerometer FOC data
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_get_acc_foc(struct bhy2_data_xyz *acc_foc, struct bhy2_dev *dev);

/*!
 * @brief To set the Accelerometer Power Mode
 *
 * @param[in] Physical Accelerometer Power Mode data
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_set_acc_power_mode(const uint8_t *acc_pwm, struct bhy2_dev *dev);

/*!
 * @brief To get the Accelerometer Power Mode
 *
 * @param[out] Physical Accelerometer Power Mode data
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_get_acc_power_mode(uint8_t *acc_pwm, struct bhy2_dev *dev);

/*!
* @brief To set the Gyroscope Fast Offset Calibration
*
* @param[in] Physical Gyroscope FOC data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_set_gyro_foc(const struct bhy2_data_xyz *gyro_foc, struct bhy2_dev *dev);

/*!
* @brief To get the Gyroscope Fast Offset Calibration
*
* @param[out] Physical Gyroscope FOC data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_get_gyro_foc(struct bhy2_data_xyz *gyro_foc, struct bhy2_dev *dev);

/*!
* @brief To set the Gyroscope OIS
*
* @param[in] Physical Gyroscope OIS data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_set_gyro_ois(const uint8_t *gyro_ois, struct bhy2_dev *dev);

/*!
* @brief To get the Gyroscope OIS
*
* @param[out] Physical Gyroscope OIS data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_get_gyro_ois(uint8_t *gyro_ois, struct bhy2_dev *dev);

/*!
* @brief To set the Gyroscope Fast Startup
*
* @param[in] Physical Gyroscope Fast Startup data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_set_gyro_fast_startup(const uint8_t *gyro_fs, struct bhy2_dev *dev);

/*!
* @brief To get the Gyroscope Fast Startup
*
* @param[out] Physical Gyroscope Fast Startup data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_get_gyro_fast_startup(uint8_t *gyro_fs, struct bhy2_dev *dev);

/*!
* @brief To set the Gyroscope CRT
*
* @param[in] Physical Gyroscope CRT data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_set_gyro_crt(const uint8_t *gyro_crt, struct bhy2_dev *dev);

/*!
* @brief To get the Gyroscope CRT
*
* @param[out] Physical Gyroscope CRT data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_get_gyro_crt(uint8_t *gyro_crt, struct bhy2_dev *dev);

/*!
* @brief To set the Gyroscope Power Mode
*
* @param[in] Physical Gyroscope Power Mode data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_set_gyro_power_mode(const uint8_t *gyro_pwm, struct bhy2_dev *dev);

/*!
* @brief To get the Gyroscope Power Mode
*
* @param[out] Physical Gyroscope Power Mode data
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_get_gyro_power_mode(uint8_t *gyro_pwm, struct bhy2_dev *dev);

/*!
* @brief To set the Gyroscope Timer Auto Trim state
*
* @param[in] Physical Gyroscope Timer Auto Trim state
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_set_gyro_timer_auto_trim(const uint8_t *gyro_tat, struct bhy2_dev *dev);

/*!
* @brief To get the Gyroscope Timer Auto Trim status
*
* @param[out] Physical Gyroscope Timer Auto Trim state
* @param[in] dev hub handle
*
* @return API error codes
*
*/
int8_t bhi3_get_gyro_timer_auto_trim(uint8_t *gyro_tat, struct bhy2_dev *dev);

/*!
 * @brief To set the Wrist Wear Wakeup Configuration
 *
 * @param[in] Wrist Wear Wakeup Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_set_wrist_wear_wakeup_config(const uint8_t *www_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To get the Wrist Wear Wakeup Configuration
 *
 * @param[out] Wrist Wear Wakeup Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_get_wrist_wear_wakeup_config(uint8_t *www_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To set the Any Motion Configuration
 *
 * @param[in] Any Motion Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_set_anymotion_config(const uint8_t *any_motion_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To get the Any Motion Configuration
 *
 * @param[out] Any Motion Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_get_anymotion_config(uint8_t *any_motion_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To set the No Motion Configuration
 *
 * @param[in] No Motion Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_set_nomotion_config(const uint8_t *no_motion_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To get the No Motion Configuration
 *
 * @param[out] No Motion Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_get_nomotion_config(uint8_t *no_motion_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To set the Wrist Gesture Detection Configuration
 *
 * @param[in] Wrist Gesture Detection Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_set_wrist_gesture_detect_config(const uint8_t *wgd_cnfg, struct bhy2_dev *dev);

/*!
 * @brief To get the Wrist Gesture Detection Configuration
 *
 * @param[out] Wrist Gesture Detection Configuration
 * @param[in] dev hub handle
 *
 * @return API error codes
 *
 */
int8_t bhi3_get_wrist_gesture_detect_config(uint8_t *wgd_cnfg, struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI3_H__ */
