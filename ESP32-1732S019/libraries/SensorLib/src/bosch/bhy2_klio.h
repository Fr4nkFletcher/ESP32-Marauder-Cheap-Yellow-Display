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
* @file       bhy2_klio.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHY2_KLIO_H__
#define __BHY2_KLIO_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>

#include "bhy2.h"
#include "bhy2_klio_defs.h"

/*!
 * @brief Reads and resets current driver status.
 *
 * The driver_status will be set to one of the values in
 * @ref bhy2_klio_driver_error_state_t. This function will not return a driver
 * status of
 * @ref bhy2_klio_driver_error_state_t::KLIO_DRIVER_ERROR_OPERATION_PENDING.
 * but will retry until the pending operation is finished.
 *
 * @param[out] driver_status Pointer to uint32_t
 * @param[in] dev            Device reference.
 *
 * @return BHY2 API error codes.
 */
int8_t bhy2_klio_read_reset_driver_status(uint32_t *driver_status, struct bhy2_dev *dev);

/*!
 * @brief Read learnt pattern.
 *
 * @param[in] id pattern id (only 0 supported now)
 * @param[out] buffer buffer big enough to fit complete pattern fingerprint data
 * @param[in,out] length size of buffer, return size of actual pattern data
 * @param[in] dev Device reference
 *
 * @return BHY2 API error codes.
 *
 * After learning of a new pattern has been signaled by a sensor data event,
 * call this function to retrieve the learnt pattern.
 */
int8_t bhy2_klio_read_pattern(const uint8_t id, uint8_t *buffer, uint16_t *length, struct bhy2_dev *dev);

/*!
 * @brief Set KLIO algorithm learning and recognition state.
 *
 * @param[in] state new state for algorithm
 * @param[in] dev Device reference
 *
 * @return BHY2 API error codes.
 *
 * Writing 1 to the reset fields resets all internal algorithm state and
 * removes any patterns loaded for recognition.
 */
int8_t bhy2_klio_set_state(const bhy2_klio_sensor_state_t *state, struct bhy2_dev *dev);

/*!
 * @brief Get KLIO algorithm learning and recognition state.
 *
 * @param[out] state current state for algorithm
 * @param[in] dev Device reference
 *
 * @return BHY2 API error codes.
 *
 * The resets are always read as 0.
 */
int8_t bhy2_klio_get_state(bhy2_klio_sensor_state_t *state, struct bhy2_dev *dev);

/*!
 * @brief Write pattern to recognition algorithm.
 *
 * @param[in] idx pattern index
 * @param[in] pattern_data containing pattern
 * @param[in] size pattern size
 * @param[in] dev Device reference
 *
 * @return BHY2 API error codes.
 *
 * Write a pattern to the specified recognition index. The allowed number of
 * recognition patterns may be retrieved using the
 * @ref bhy2_klio_parameter::KLIO_PARAM_RECOGNITION_MAX_PATTERNS parameter.
 *
 * After a pattern is written, it is in disabled state and not used. It must
 * be enabled using @ref bhy2_klio_set_pattern_states() before it will be used
 * by the recognition algorithm.
 */
int8_t bhy2_klio_write_pattern(const uint8_t idx, const uint8_t *pattern_data, const uint16_t size,
                               struct bhy2_dev *dev);

/*!
 * @brief Sets the specified state for the specified pattern ids.
 *
 * @param[in] operation operation to perform
 * @param[in] pattern_ids ids of pattern(s) to perform operation on
 * @param[in] size size of pattern_ids array
 * @param[in] dev Device reference
 *
 * @return BHY2 API error codes.
 */
int8_t bhy2_klio_set_pattern_states(const bhy2_klio_pattern_state_t operation,
                                    const uint8_t *pattern_ids,
                                    const uint16_t size,
                                    struct bhy2_dev *dev);

/*!
 * @brief Calculates similarity score between two patterns.
 *
 * The similariy between patterns is a meassure of the similarity between the
 * performed activities or exercises. If the two patterns represent the same
 * excercise the similarity will be close to 1.0, and if they are for very
 * different excercises, the similarity will be negative.
 *
 * @param[in]  first_pattern  Buffer containing first pattern.
 * @param[in]  second_pattern Buffer containing second pattern.
 * @param[in]  size           Pattern size.
 * @param[out] similarity     Similarity score.
 * @param[in]  dev            Device reference.
 *
 * @return BHY2 API error codes.
 */
int8_t bhy2_klio_similarity_score(const uint8_t *first_pattern,
                                  const uint8_t *second_pattern,
                                  const uint16_t size,
                                  float *similarity,
                                  struct bhy2_dev *dev);

/*!
 * @brief Calculates similarity score between multiple patterns.
 *
 * This is more efficient than bhy2_klio_similarity_score() since if the patterns
 * are already available on the bhy2, they need not be uploaded again to
 * perform the comparison. And several comparisons may be performed with one
 * API call.
 *
 * @param[in]  idx      Reference pattern.
 * @param[in]  indexes    Buffer containing indexes of patterns to compare reference pattern with.
 * @param[in]  count      Number of patterns in indexes buffer.
 * @param[out] similarity Similarity scores (one score for each pattern in indexes buffer).
 * @param[in]  dev        Device reference.
 *
 * @return BHY2 API error codes.
 */
int8_t bhy2_klio_similarity_score_multiple(const uint8_t idx,
                                           const uint8_t *indexes,
                                           const uint8_t count,
                                           float *similarity,
                                           struct bhy2_dev *dev);

/*!
 * @brief Set KLIO algorithm parameter
 *
 * @param[in] id             Parameter to set.
 * @param[in] parameter_data Buffer containing new parameter value.
 * @param[in] size           Parameter size.
 * @param[in] dev            Device reference.
 *
 * @return BHY2 API error codes.
 */
int8_t bhy2_klio_set_parameter(const bhy2_klio_parameter_t id,
                               const void *parameter_data,
                               const uint16_t size,
                               struct bhy2_dev *dev);

/*!
 * @brief Get KLIO algorithm parameter
 *
 * @param[in]     id             Parameter id.
 * @param[out]    parameter_data Buffer to store parameter data.
 * @param[in,out] size           Parameter data size.
 * @param[in]     dev            Device reference.
 *
 * @return BHY2 API error codes.
 *
 * The expected type or buffer size of parameter_data for respective parameter
 * id is specified in the @ref bhy2_klio_parameter_t enum.
 */
int8_t bhy2_klio_get_parameter(const bhy2_klio_parameter_t id,
                               uint8_t *parameter_data,
                               uint16_t *size,
                               struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif
