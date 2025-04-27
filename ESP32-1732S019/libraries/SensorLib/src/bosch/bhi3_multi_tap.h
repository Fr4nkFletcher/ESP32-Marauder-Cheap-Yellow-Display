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
* @file       bhi3_multi_tap.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef _BHY2_MULTI_TAP_H_
#define _BHY2_MULTI_TAP_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>

#include "bhi3.h"
#include "bhi3_multi_tap_defs.h"

/*!
 * @brief Parsing the fifo data to MULTI_TAP output structure format
 *
 * @param[in] Multi Tap data
 * @param[out] buffer to store parameter data
 *
 * @return  API error codes
 *
 */
int8_t bhi3_multi_tap_parse_data(const uint8_t *data, uint8_t *output);

/*!
 * @brief To set the MULTI_TAP configuration parameters
 * @param[out] buffer to store parameter data
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhi3_multi_tap_set_config(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the MULTI_TAP configuration parameters
 *
 * @param[out] buffer to store parameter data
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhi3_multi_tap_get_config(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To set the MULTI_TAP Detector configuration parameters
 * @param[out] buffer to store parameter data
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhi3_multi_tap_detector_set_config(const void *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the MULTI_TAP Detector configuration parameters
 *
 * @param[out] buffer to store parameter data
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhi3_multi_tap_detector_get_config(const void *buffer, struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHY2_MULTI_TAP_H_ */
