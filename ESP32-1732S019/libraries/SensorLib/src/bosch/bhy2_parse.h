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
* @file       bhy2_parse.h
* @date       2023-03-24
* @version    v1.6.0
*
*/
#ifndef __BHY2_PARSE_H__
#define __BHY2_PARSE_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhy2.h"

/**
 * @brief Function to parse FIFO frame data into temperature
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] temperature  : Reference to the data buffer to store temperature in degree C
 */
void bhy2_parse_temperature_celsius(const uint8_t *data, bhy2_float *temperature);

/**
 * @brief Function to parse FIFO frame data into humidity
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] humidity : Reference to the data buffer to store humidity in %
 */
void bhy2_parse_humidity(const uint8_t *data, bhy2_float *humidity);

/**
 * @brief Function to parse FIFO frame data into barometric pressure
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] pressure : Reference to the data buffer to store pressure in Pascals
 */
void bhy2_parse_pressure(const uint8_t *data, bhy2_float *pressure);

/**
 * @brief Function to parse FIFO frame data into altitude
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] altitude : Reference to the data buffer to store altitude
 */
void bhy2_parse_altitude(const uint8_t *data, bhy2_float *altitude);

/**
 * @brief Function to parse FIFO frame data into quaternion
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] quaternion   : Reference to the data buffer to store quaternion
 */
void bhy2_parse_quaternion(const uint8_t *data, struct bhy2_data_quaternion *quaternion);

/**
 * @brief Function to parse FIFO frame data into orientation
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] orientation  : Reference to the data buffer to store orientation
 */
void bhy2_parse_orientation(const uint8_t *data, struct bhy2_data_orientation *orientation);

/**
 * @brief Function to parse FIFO frame data into 3 axes vector
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] vector   : Reference to the data buffer to store vector
 */
void bhy2_parse_xyz(const uint8_t *data, struct bhy2_data_xyz *vector);

/**
 * @brief Function to parse FIFO frame data into step counter
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @return[out]         : Reference to the data buffer to step counter
 */
uint32_t bhy2_parse_step_counter(const uint8_t *data);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY2_PARSE_H__ */
