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
* @file       bhy2_klio.c
* @date       2023-03-24
* @version    v1.6.0
*
*/

/*********************************************************************/
/* system header files */
#include <string.h>
#include <stdio.h>

/*********************************************************************/
/* BHY2 SensorAPI header files */
#include "bhy2.h"

/*********************************************************************/
/* own header files */
#include "bhy2_klio.h"

int8_t bhy2_klio_read_pattern(const uint8_t id, uint8_t *buffer, uint16_t *length, struct bhy2_dev *dev)
{
    int8_t rslt;
    uint32_t ret_len;
    bhy2_klio_pattern_transfer_t pattern = { 0 };

    if (!buffer || !length || !dev)
    {
        return BHY2_E_NULL_PTR;
    }

    if (id != 0 || *length < sizeof(pattern.pattern_data))
    {
        return BHY2_E_INVALID_PARAM;
    }

    rslt = bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN), (uint8_t*)&pattern, sizeof(pattern), &ret_len, dev);

    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    if (ret_len != sizeof(pattern))
    {
        return BHY2_E_INVALID_PARAM;
    }

    memcpy(buffer, pattern.pattern_data, pattern.full_size);
    *length = pattern.full_size;

    return rslt;
}

int8_t bhy2_klio_read_reset_driver_status(uint32_t *klio_driver_status, struct bhy2_dev *dev)
{
    uint32_t ret_len = 0;
    int8_t rslt;

    if (!klio_driver_status || !dev)
    {
        return BHY2_E_NULL_PTR;
    }

    do
    {
        rslt =
            bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_DRIVER_STATUS),
                               (uint8_t*)klio_driver_status,
                               sizeof(*klio_driver_status),
                               &ret_len,
                               dev);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }
    } while (*klio_driver_status == KLIO_DRIVER_ERROR_OPERATION_PENDING);

    return rslt;
}

int8_t bhy2_klio_write_pattern(const uint8_t idx, const uint8_t *pattern_data, const uint16_t size,
                               struct bhy2_dev *dev)
{
    bhy2_klio_pattern_transfer_t pattern = {
        .block_id = 0, .block_size = (uint8_t)size, .full_size = size, .pattern_id = idx, .pattern_data = { 0 }
    };

    if (!dev || size > sizeof(pattern.pattern_data))
    {
        return BHY2_E_INVALID_PARAM;
    }

    memcpy(pattern.pattern_data, pattern_data, size);

    return bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN), (const uint8_t*)&pattern, sizeof(pattern), dev);
}

int8_t bhy2_klio_set_pattern_states(const bhy2_klio_pattern_state_t operation,
                                    const uint8_t *pattern_ids,
                                    const uint16_t count,
                                    struct bhy2_dev *dev)
{
    uint32_t buffer_size = sizeof(bhy2_klio_pattern_state_op_t) + count;
    uint8_t buffer[buffer_size];

    if (!dev || count == 0)
    {
        return BHY2_E_INVALID_PARAM;
    }

    buffer[1] = (uint8_t)count;
    buffer[0] = operation;

    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i + 2] = pattern_ids[i];
    }

    return bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_STATE), buffer, buffer_size, dev);
}

int8_t bhy2_klio_set_state(const bhy2_klio_sensor_state_t *state, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer[4];

    if (!dev || !state)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        buffer[0] = state->learning_enabled;
        buffer[1] = state->learning_reset;
        buffer[2] = state->recognition_enabled;
        buffer[3] = state->recognition_reset;

        rslt = bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGORITHM_STATE), buffer, sizeof(buffer), dev);
    }

    return rslt;
}

int8_t bhy2_klio_get_state(bhy2_klio_sensor_state_t *state, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (!dev || !state)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        uint8_t buffer[4];
        uint32_t ret_len;

        rslt = bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGORITHM_STATE), buffer, sizeof(buffer), &ret_len, dev);
        if (rslt == BHY2_OK && ret_len == sizeof(buffer))
        {
            state->learning_enabled = buffer[0];
            state->learning_reset = buffer[1];
            state->recognition_enabled = buffer[2];
            state->recognition_reset = buffer[3];
        }
    }

    return rslt;
}

int8_t bhy2_klio_similarity_score(const uint8_t *first_pattern,
                                  const uint8_t *second_pattern,
                                  const uint16_t size,
                                  float *similarity,
                                  struct bhy2_dev *dev)
{
    uint32_t klio_driver_status = 0;
    int8_t rslt;
    uint32_t ret_len;
    bhy2_klio_pattern_transfer_t pattern1 = { 0 }, pattern2 = { 0 };
    uint8_t buffer[4];

    if (!dev || size > sizeof(pattern1.pattern_data))
    {
        return BHY2_E_INVALID_PARAM;
    }

    pattern1.block_size = pattern2.block_size = (uint8_t)size;
    pattern1.full_size = pattern2.full_size = size;
    memcpy(pattern1.pattern_data, first_pattern, size);
    memcpy(pattern2.pattern_data, second_pattern, size);

    rslt =
        bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY), (uint8_t*)&pattern1, sizeof(pattern1), dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    rslt =
        bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY), (uint8_t*)&pattern2, sizeof(pattern2), dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    /* Wait for similarity calculation to complete. */
    rslt = bhy2_klio_read_reset_driver_status(&klio_driver_status, dev);
    if (rslt != BHY2_OK || klio_driver_status != KLIO_DRIVER_ERROR_NONE)
    {
        return BHY2_E_INVALID_PARAM;
    }

    rslt = bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY), buffer, sizeof(buffer), &ret_len, dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    if (ret_len != sizeof(buffer))
    {
        return BHY2_E_INVALID_PARAM;
    }

    memcpy(similarity, buffer, sizeof(*similarity));

    return BHY2_OK;
}

int8_t bhy2_klio_similarity_score_multiple(const uint8_t idx,
                                           const uint8_t *indexes,
                                           const uint8_t count,
                                           float *similarity,
                                           struct bhy2_dev *dev)
{
    uint32_t klio_driver_status;
    int8_t rslt;
    uint32_t ret_len;
    uint32_t buffer_size = sizeof(bhy2_klio_similarity_calculation_t) + count;

    buffer_size = BHY2_ROUND_WORD_HIGHER(buffer_size);
    uint8_t buffer[256] = { 0 };

    if (!indexes || !similarity || !dev)
    {
        return BHY2_E_NULL_PTR;
    }

    /* buffer[0-3] values are initialized to zero's to distinguish from regular pattern comparison*/
    buffer[4] = idx;
    buffer[5] = count;
    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i + 6] = indexes[i];
    }

    /* Start similarity calculations. */
    rslt = bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY), (uint8_t*)buffer, buffer_size, dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    /* Wait for similarity calculation to complete. */
    rslt = bhy2_klio_read_reset_driver_status(&klio_driver_status, dev);
    if (rslt != BHY2_OK || klio_driver_status != KLIO_DRIVER_ERROR_NONE)
    {
        return BHY2_E_INVALID_PARAM;
    }

    /* Read similarity results. */
    rslt = bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY),
                              (uint8_t *)similarity,
                              sizeof(float) * count,
                              &ret_len,
                              dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    return BHY2_OK;
}

int8_t bhy2_klio_set_parameter(const bhy2_klio_parameter_t id,
                               const void *parameter_data,
                               const uint16_t size,
                               struct bhy2_dev *dev)
{
    bhy2_klio_parameter_wrapper_t buffer;

    buffer.id = id;
    buffer.flags = 0;
    buffer.size = (uint8_t)size;
    memcpy(buffer.payload.data, parameter_data, size);

    /* Size is 3 bytes header and buffer.size then rounded up to a four byte multiple. */
    return bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER),
                              (uint8_t*)&buffer,
                              (3 + buffer.size + 3) & ~3,
                              dev);
}

int8_t bhy2_klio_get_parameter(const bhy2_klio_parameter_t id,
                               uint8_t *parameter_data,
                               uint16_t *size,
                               struct bhy2_dev *dev)
{
    uint32_t ret_len;
    uint8_t buffer[260] = { 0 };

    buffer[0] = id;
    int8_t rslt;

    /* Size is 3 bytes header and buffer.size then rounded up to a four byte multiple. */
    rslt = bhy2_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER), buffer, (3 + buffer[2] + 3) & ~3, dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    rslt = bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER), buffer, sizeof(buffer), &ret_len, dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    if (ret_len < 3 || buffer[2] > *size)
    {
        return BHY2_E_INVALID_PARAM;
    }

    for (uint16_t i = 3, j = 0; i < buffer[2] + 3; i++, j++)
    {
        parameter_data[j] = buffer[i];
    }

    *size = buffer[2];

    return BHY2_OK;
}
