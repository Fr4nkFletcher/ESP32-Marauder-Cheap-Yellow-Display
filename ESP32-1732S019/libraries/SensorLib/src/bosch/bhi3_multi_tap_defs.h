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
* @file       bhi3_multi_tap_defs.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef _BHI3_MULTI_TAP_DEFS_H_
#define _BHI3_MULTI_TAP_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <string.h>

/*! Sensor ID for Multi-Tap */
#define BHI3_SENSOR_ID_MULTI_TAP                   UINT8_C(153)

/*! Multi-Tap Parameter Page Base Address*/
#define BHI3_MULTI_TAP_PAGE                        UINT16_C(0x0D)

/*! Multi-Tap Configuration pages */
#define BHI3_MULTI_TAP_ENABLE_PARAM                0x01
#define BHI3_MULTI_TAP_DETECTOR_CONFIG_PARAM       0x02
#define BHI3_MULTI_TAP_PARAM(id)                   (((BHI3_MULTI_TAP_PAGE) << 8) | (id))

#define BHI3_MULTI_TAP_ENABLE_LENGTH               UINT8_C(4)
#define BHI3_MULTI_TAP_DETECTOR_CONFIG_LENGTH      UINT8_C(6)

#define BHI3_MULTI_TAP_AXIS_SEL_X                  UINT8_C(0)
#define BHI3_MULTI_TAP_AXIS_SEL_Y                  UINT8_C(1)
#define BHI3_MULTI_TAP_AXIS_SEL_Z                  UINT8_C(2)

#define BHI3_MULTI_TAP_WAIT_TIMEOUT_DISABLE        UINT8_C(0)
#define BHI3_MULTI_TAP_WAIT_TIMEOUT_ENABLE         UINT8_C(1)

#define BHI3_MULTI_TAP_FILTER_MODE_SENSITIVE       UINT8_C(0)
#define BHI3_MULTI_TAP_FILTER_MODE_NORMAL          UINT8_C(1)
#define BHI3_MULTI_TAP_FILTER_MODE_ROBUST          UINT8_C(2)

#define BHI3_SINGLE_TAP_AXIS_SEL_MASK              0x03
#define BHI3_SINGLE_TAP_WAIT_TIMEOUT_MASK          0x04
#define BHI3_SINGLE_TAP_MAX_PEAKS_FOR_TAP_MASK     0x38
#define BHI3_SINGLE_TAP_FILTER_MODE_MASK           0xC0

#define BHI3_DOUBLE_TAP_TAP_PEAK_DUR_MASK          0x03FF
#define BHI3_DOUBLE_TAP_MAX_GES_DUR_MASK           0xFC00

#define BHI3_TRIPLE_TAP_MAX_DUR_BW_PEAKS_MASK      0x0F
#define BHI3_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_MASK    0xF0
#define BHI3_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_MASK   0x0F
#define BHI3_TRIPLE_TAP_QT_TM_AFTER_GESTURE_MASK   0xF0

#define BHI3_SINGLE_TAP_AXIS_SEL_SHIFT             UINT8_C(0)
#define BHI3_SINGLE_TAP_WAIT_TIMEOUT_SHIFT         UINT8_C(2)
#define BHI3_SINGLE_TAP_MAX_PEAKS_FOR_TAP_SHIFT    UINT8_C(3)
#define BHI3_SINGLE_TAP_FILTER_MODE_SHIFT          UINT8_C(6)

#define BHI3_DOUBLE_TAP_TAP_PEAK_DUR_SHIFT         UINT8_C(0)
#define BHI3_DOUBLE_TAP_MAX_GES_DUR_SHIFT          UINT8_C(10)

#define BHI3_TRIPLE_TAP_MAX_DUR_BW_PEAKS_SHIFT     UINT8_C(0)
#define BHI3_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_SHIFT   UINT8_C(4)
#define BHI3_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_SHIFT  UINT8_C(0)
#define BHI3_TRIPLE_TAP_QT_TM_AFTER_GESTURE_SHIFT  UINT8_C(4)

/*!
 * Single Tap Configuration.
 */
typedef union bhi3_singletap_detector_settings
{
    uint16_t as_uint16;
    struct
    {
        uint16_t axis_sel                : 2; /* Tap axis selection */
        uint16_t wait_for_timeout        : 1; /* Wait for gesture confirmation */
        uint16_t max_peaks_for_tap       : 3; /* Maximum number of peaks that can occur when a tap is done
                                                           * */
        uint16_t mode                    : 2; /* Filter configuration for various detection mode: SENSITIVE, NORMAL and
                                               * ROBUST */
        uint16_t reserved8               : 8;
    } as_s;
} __attribute__ ((packed)) bhi3_singletap_detector_settings_t;

/*!
 * Double Tap Configuration.
 */
typedef union bhi3_doubletap_detector_settings
{
    uint16_t as_uint16;
    struct
    {
        uint16_t tap_peak_thres          : 10; /* Minimum threshold for peak detection */
        uint16_t max_gesture_dur         : 6; /* Maximum time duration from first tap within which matching
                                               * tap/s should happen to be detected as double/triple tap */
    } as_s;
} __attribute__ ((packed)) bhi3_doubletap_detector_settings_t;

/*!
 * Triple Tap Configuration.
 */
typedef union bhi3_tripletap_detector_settings
{
    uint16_t as_uint16;
    struct
    {
        uint16_t max_dur_between_peaks       : 4; /* Maximum time duration within which matching peaks of tap
                                                   * should occur */
        uint16_t tap_shock_settling_dur      : 4; /* Maximum duration to wait for tap shock settling */
        uint16_t min_quite_dur_between_taps  : 4; /* Minimum quite time between detection of consecutive taps
                                                   * of double/triple taps*/
        uint16_t quite_time_after_gesture    : 4; /* Minimum quite time between detection of 2 consecutive
                                                   * selected gesture */
    } as_s;
} __attribute__ ((packed)) bhi3_tripletap_detector_settings_t;

/*!
 * Multi Tap Configuration.
 */
typedef struct bhi3_multi_tap_detector
{
    bhi3_singletap_detector_settings_t stap_setting;
    bhi3_doubletap_detector_settings_t dtap_setting;
    bhi3_tripletap_detector_settings_t ttap_setting;
} __attribute__ ((packed)) bhi3_multi_tap_detector_t;

/*!
 * Multi Tap Setting.
 */
enum bhi3_multi_tap_val {
    NO_TAP,
    SINGLE_TAP,
    DOUBLE_TAP,
    DOUBLE_SINGLE_TAP,
    TRIPLE_TAP,
    TRIPLE_SINGLE_TAP,
    TRIPLE_DOUBLE_TAP,
    TRIPLE_DOUBLE_SINGLE_TAP
};

/*!
 * Multi Tap Output.
 */
static const char * const bhi3_multi_tap_string_out[] = {
    [NO_TAP] = "NO_TAP", [SINGLE_TAP] = "SINGLE_TAP", [DOUBLE_TAP] = "DOUBLE_TAP",
    [DOUBLE_SINGLE_TAP] = "DOUBLE_SINGLE_TAP", [TRIPLE_TAP] = "TRIPLE_TAP", [TRIPLE_SINGLE_TAP] = "TRIPLE_SINGLE_TAP",
    [TRIPLE_DOUBLE_TAP] = "TRIPLE_DOUBLE_TAP", [TRIPLE_DOUBLE_SINGLE_TAP] = "TRIPLE_DOUBLE_SINGLE_TAP"
}; /*lint -e528 */

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI3_MULTI_TAP_DEFS_H_ */
