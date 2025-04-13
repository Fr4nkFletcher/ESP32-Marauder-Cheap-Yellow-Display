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
* @file       bhi3_multi_tap.c
* @date       2023-03-24
* @version    v1.6.0
*
*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdio.h>

/*********************************************************************/
/* BHI3 SensorAPI header files */
/*********************************************************************/
#include "bhi3.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhi3_multi_tap.h"

/*lint -e506, -e778*/

/*!
 * @brief This API writes to the configuration parameter
 *
 * @param[in] buffer to store Multi Tap configuration
 * @param[out] dev hub handle.
 *
 * @return rslt execution result.
 */
int8_t bhi3_multi_tap_set_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the Multi Tap configuration parameter */
        rslt = bhy2_set_parameter(BHI3_MULTI_TAP_PARAM(BHI3_MULTI_TAP_ENABLE_PARAM),
                                  (uint8_t*)buffer,
                                  BHI3_MULTI_TAP_ENABLE_LENGTH,
                                  dev);
    }

    return rslt;
}

/*!
 * @brief To get the MULTI-TAP configuration parameters
 *
 * @param[out] buffer to store Multi Tap configuration
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhi3_multi_tap_get_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t ret_length;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the Multi Tap configuration from the parameter page*/
        rslt =
            bhy2_get_parameter(BHI3_MULTI_TAP_PARAM(BHI3_MULTI_TAP_ENABLE_PARAM),
                               (uint8_t *)buffer,
                               BHY2_LE24MUL(BHI3_MULTI_TAP_ENABLE_LENGTH),
                               &ret_length,
                               dev);

        if (rslt != BHY2_OK)
        {
            /*! Invalid number of parameters readout */
            rslt = BHY2_E_INVALID_EVENT_SIZE;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes to the tap detector configuration parameter
 *
 * @param[in] buffer to store Multi Tap Detector configuration
 * @param[out] dev hub handle.
 *
 * @return rslt execution result.
 */
int8_t bhi3_multi_tap_detector_set_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the Multi Tap configuration parameter */
        rslt = bhy2_set_parameter(BHI3_MULTI_TAP_PARAM(BHI3_MULTI_TAP_DETECTOR_CONFIG_PARAM),
                                  (uint8_t*)buffer,
                                  BHI3_MULTI_TAP_DETECTOR_CONFIG_LENGTH,
                                  dev);
    }

    return rslt;
}

/*!
 * @brief To get the MULTI-TAP Detector configuration parameters
 *
 * @param[out] buffer to store Multi Tap Detector configuration
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhi3_multi_tap_detector_get_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t ret_length;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the Multi Tap configuration from the parameter page*/
        rslt = bhy2_get_parameter(BHI3_MULTI_TAP_PARAM(BHI3_MULTI_TAP_DETECTOR_CONFIG_PARAM),
                                  (uint8_t *)buffer,
                                  BHY2_LE24MUL(BHI3_MULTI_TAP_DETECTOR_CONFIG_LENGTH),
                                  &ret_length,
                                  dev);

        if (rslt != BHY2_OK)
        {
            /*! Invalid number of parameters readout */
            rslt = BHY2_E_INVALID_EVENT_SIZE;
        }
    }

    return rslt;
}

/*!
 * @brief Parsing the fifo data to Multi Tap output format
 *
 * @param[in] Multi Tap data
 * @param[out] buffer to store parameter data
 *
 * @return  API error codes
 *
 */
int8_t bhi3_multi_tap_parse_data(const uint8_t *data, uint8_t* output)
{
    int8_t rslt = BHY2_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        *output = *data;
    }

    return rslt;
}
