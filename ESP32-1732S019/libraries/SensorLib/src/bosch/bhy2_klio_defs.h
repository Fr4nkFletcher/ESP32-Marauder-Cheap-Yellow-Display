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
* @file       bhy2_klio_defs.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHY2_KLIO_DEFS_H__
#define __BHY2_KLIO_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>

#define BHY2_SENSOR_ID_KLIO      UINT8_C(112)
#define BHY2_SENSOR_ID_KLIO_LOG  UINT8_C(127)

#ifndef BHY2_KLIO_PAGE
#define BHY2_KLIO_PAGE           UINT16_C(9)
#endif
#define KLIO_PARAM(id)           (((BHY2_KLIO_PAGE) << 8) | (id))

/*!
 * @enum bhy2_klio_parameter
 *
 * KLIO algorithm parameters
 *
 * @var bhy2_klio_parameter::KLIO_PARAM_ALGORITHM_VERSION
 * Algorithm version. (read)
 * @code{.c} char version[244]; @endcode
 *
 * @var bhy2_klio_parameter::KLIO_PARAM_RECOGNITION_RESPONSIVNESS
 * The approximate number of cycles / repetitions for recognition to recognize
 * an activity. (read / write)
 * @code{.c} float cycle_fraction; @endcode
 *
 * @var bhy2_klio_parameter::KLIO_PARAM_PATTERN_BLOB_SIZE
 * Pattern blob size in bytes. (read)
 * @code{.c} uint16_t pattern_blob_size; @endcode
 *
 * @var bhy2_klio_parameter::KLIO_PARAM_RECOGNITION_MAX_PATTERNS
 * Read Maximum number of patterns. (read)
 * @code{.c} uint16_t recognition_max_patterns; @endcode
 *
 * @var bhy2_klio_parameter::KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT
 * Flag if insignificant movements should be ignored in learning. Value 0 or 1. (read / write)
 * @code{.c} uint8_t ignore_insig_movement; @endcode
 */
typedef enum
{
    KLIO_PARAM_ALGORITHM_VERSION = 0,
    KLIO_PARAM_RECOGNITION_SENSITIVITY = 1,
    KLIO_PARAM_RECOGNITION_RESPONSIVNESS = 2,
    KLIO_PARAM_LEARNING_SPEED = 3,
    KLIO_PARAM_LEARNING_SIMILARITY_THRESHOLD = 4,
    KLIO_PARAM_LEARNING_FREQUENCIES = 5,
    KLIO_PARAM_LEARNING_PROGRESS = 6,
    KLIO_PARAM_PATTERN_BLOB_SIZE = 7,
    KLIO_PARAM_RECOGNITION_MAX_PATTERNS = 8,
    KLIO_PARAM_INPUT_SENSORS = 9,
    KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT = 10
} bhy2_klio_parameter_t;

/*!
 *
 * @brief Convenience wrapper used in bhy2_klio.c algo parameter wrapper.
 *
 */
typedef struct
{
    uint8_t id;
    uint8_t flags;
    uint8_t size;
    union
    {
        uint8_t data[244];
        char version[244];
        float cycle_fraction;
        uint16_t pattern_blob_size;
        uint16_t recognition_max_patterns;
        uint8_t ignore_insig_movement;
    } BHY2_PACKED payload;
} BHY2_PACKED bhy2_klio_parameter_wrapper_t;

/*!
 *
 * @brief bhy2 klio state structure
 *
 */
typedef struct
{
    uint8_t learning_enabled; /*!< 0 - disable learning, 1 - enable learning. */
    uint8_t learning_reset; /*!< 0 - nop, 1 - reset learning. Always read as 0. */
    uint8_t recognition_enabled; /*!< 0 - disable recognition, 1 - enable recognition. */
    uint8_t recognition_reset; /*!< 0 - nop, 1 - reset learning. Always read as 0. */
} BHY2_PACKED bhy2_klio_sensor_state_t;

typedef struct
{
    uint8_t instance;

    /*!
     * A value of -1 means no new learning has occurred. If the value is >= 0,
     * then a new pattern has been learnt, and reading of this pattern may be
     * performed.
     */
    int8_t index;

    /*!
     * While learning a new pattern, this field counts from 0 to 100. When 100 is
     * reached a new pattern will be learnt. If learning is interrupted, this
     * progress will return to 0, and change reason will be set to indicate
     * why learning was interrupted.
     */
    uint8_t progress;

    /*!
     * | Value | Description                                                            |
     * |:-----:|:-----------------------------------------------------------------------|
     * | 0     | Learning is progressing.                                               |
     * | 1     | Learning was interrupted by a non-repetitive activity.                 |
     * | 2     | Learning was interrupted because no significant movement was detected. |
     */
    uint8_t change_reason;
} BHY2_PACKED bhy2_klio_sensor_frame_learning_t;

typedef struct
{
    uint8_t instance;

    /*! The index of the recognized activity. 255 means no activity was
     * recognized. */
    uint8_t index;

    /*! The current repetition count of the recognized activity. */
    float count;
} BHY2_PACKED bhy2_klio_sensor_frame_recognition_t;

/*!
 *
 * @brief bhy2 klio combined data structure
 *
 * When the algorithm generates a new data frame, it is sent in this structure.
 *
 */
typedef struct
{
    bhy2_klio_sensor_frame_learning_t learn;
    bhy2_klio_sensor_frame_recognition_t recognize;
} BHY2_PACKED bhy2_klio_sensor_frame_t;

/*!
 *
 * @brief bhy2 klio sensor log frame
 *
 * When the algorithm consumes a new set of samples, it is sent in this structure.
 *
 */
typedef struct
{
    float timestamp;
    float accel[3];
    float gyro[3];
} BHY2_PACKED bhy2_klio_log_frame_t;

/*!
 *
 * @brief bhy2 klio pattern transfer format
 *
 */
typedef struct
{
    uint8_t block_id; /*!< Write as 0. */
    uint8_t block_size; /*!< Write as 0. */

    /*! Size of pattern_data in bytes. The size of patterns may be obtained
    using the @ref bhy2_klio_parameter::KLIO_PARAM_PATTERN_BLOB_SIZE
    parameter. */
    uint16_t full_size;

    /*! Id of pattern to write. */
    uint8_t pattern_id;

    /*! Actual pattern data describing the exercise. */
    uint8_t pattern_data[244];
} BHY2_PACKED bhy2_klio_pattern_transfer_t;

/*!
 *
 * @brief Convenience wrapper used by @ref bhy2_klio_similarity_score_multiple().
 *
 */
typedef struct
{
    uint32_t zero;
    uint8_t index;
    uint8_t count;
    uint8_t *indexes;
} BHY2_PACKED bhy2_klio_similarity_calculation_t;

/*!
 *
 * @brief Convenience wrapper used by @ref bhy2_klio_set_pattern_states().
 *
 */
typedef struct
{
    uint8_t operation;
    uint8_t count;
    uint8_t *indexes;
} BHY2_PACKED bhy2_klio_pattern_state_op_t;

/*!
 * KLIO pattern state
 */
typedef enum
{
    /*!< disable pattern */
    KLIO_PATTERN_STATE_DISABLE = 0,

    /*!< enable pattern */
    KLIO_PATTERN_STATE_ENABLE = 1,

    /*!< switch hand */
    KLIO_PATTERN_STATE_SWITCH_HAND = 2,

    /*!< disable adaptive pattern */
    KLIO_PATTERN_STATE_AP_DISABLE = 3
} bhy2_klio_pattern_state_t;

/*!
 * @brief KLIO host interface parameter identifiers.
 *
 * The parameter id used when calling the BHY2 functions directly. E.g. to read the driver status this may be used.
 *
 * ~~~{.c}
 * uint32_t klio_driver_status = 0;
 * uint32_t ret_len = 0;
 * int8_t rslt = bhy2_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_DRIVER_STATUS),
 *                                  (uint8_t*)&klio_driver_status,
 *                                  sizeof(klio_driver_status),
 *                                  &ret_len,
 *                                  dev);
 * if (rslt != BHY2_OK)
 * {
 *   return rslt;
 * }
 * ~~~
 */
typedef enum
{
    /*!< Read and write algorighm state. */
    KLIO_HIF_PARAM_ALGORITHM_STATE = 0,

    /*!< Read and write patterns. */
    KLIO_HIF_PARAM_PATTERN = 1,

    /*!< Read and write algorithm parameters. */
    KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER = 2,
    KLIO_HIF_PARAM_STATISTIC = 3,
    KLIO_HIF_PARAM_PATTERN_STATE = 4,

    /*!< Perform pattern similarity. */
    KLIO_HIF_PARAM_PATTERN_SIMILARITY = 5,
    KLIO_HIF_PARAM_PATTERN_COMBINING = 6,

    /*!< Read driver status. */
    KLIO_HIF_PARAM_DRIVER_STATUS = 7
} bhy2_klio_hif_parameter_t;

/*!
 * KLIO driver error codes
 */
typedef enum
{
    /*!< No error. */
    KLIO_DRIVER_ERROR_NONE = 0,

    /*!< Invalid parameter. */
    KLIO_DRIVER_ERROR_INVALID_PARAMETER = 1,

    /*!< Parameter out of range. */
    KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE = 2,

    /*!
     * Invalid pattern operation. E.g. trying to read a learnt pattern when no
     * pattern has been learnt, or trying to write an invalid pattern.
     */
    KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION = 3,

    /*!< Not implemented. */
    KLIO_DRIVER_ERROR_NOT_IMPLEMENTED = 4,

    /*!< Insufficient buffer size */
    KLIO_DRIVER_ERROR_BUFSIZE = 5,

    /*!< Internal error. */
    KLIO_DRIVER_ERROR_INTERNAL = 6,

    /*!< Undefined error */
    KLIO_DRIVER_ERROR_UNDEFINED = 7,

    /*!
     * Previous operation is still progressing. Read driver status again until
     * operation is finished.
     */
    KLIO_DRIVER_ERROR_OPERATION_PENDING = 8
} bhy2_klio_driver_error_state_t;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY2_KLIO_DEFS_H__ */
