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
* @file       bhy2_swim.c
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
/* BHY2 SensorAPI header files */
/*********************************************************************/
#include "bhy2.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhy2_swim.h"

/*!
 * @brief This API writes to the configuration parameter
 * Config parameters are pool length and handedness
 *
 * @param[in] buffer to store swim configuration
 * @param[out] dev hub handle.
 *
 * @return rslt execution result.
 */
int8_t bhy2_swim_set_config(const void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    /*! Length of the configuration is 4 bytes */
    uint32_t length = BHY2_SWIM_DATA_LENGTH;

    /*! Swim Parameter page with offset 0 used for swim configuration*/
    uint16_t param = BHY2_SWIM_CONFIG_PARAM;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Set the swim configuration parameter */
        rslt = bhy2_set_parameter(param, (uint8_t *)buffer, length, dev);
    }

    return rslt;
}

/*!
 * @brief To get the SWIM configuration parameters like swim length and handedness
 *
 * @param[out] buffer to store swim configuration
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhy2_swim_get_config(void *buffer, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    /*! Length of the configuration is 4 bytes */
    uint32_t length = BHY2_SWIM_DATA_LENGTH;
    uint32_t ret_length;

    /*! Swim Parameter page with offset 0 used to store swim configuration*/
    uint16_t param = BHY2_SWIM_CONFIG_PARAM;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the swim configuration from the parameter page*/
        rslt = bhy2_get_parameter(param, buffer, length, &ret_length, dev);

        if ((rslt == BHY2_OK) && (ret_length != BHY2_SWIM_DATA_LENGTH))
        {
            /*! Invalid number of parameters readout */
            rslt = BHY2_E_INVALID_EVENT_SIZE;
        }
    }

    return rslt;
}

/*!
 * @brief To get the SWIM Algorithm version
 *
 * @param[out] buffer to store version data
 * @param[in] dev hub handle
 *
 * @return  status code, BHY_HIF_E_SUCCESS in case of success
 *
 */
int8_t bhy2_swim_get_version(bhy2_swim_version_t *version, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    /*! Buffer to store the swim version */
    uint8_t buffer[4];

    /*! Length of the configuration is 4 */
    uint32_t length = BHY2_SWIM_DATA_LENGTH;
    uint32_t ret_length;

    /*! Swim Parameter page with offset 1 used to store swim algorithm version*/
    uint16_t param = BHY2_SWIM_VERSION;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /*! Get the swim version from the parameter page */
        rslt = bhy2_get_parameter(param, buffer, length, &ret_length, dev);

        if ((rslt == BHY2_OK) && (ret_length != BHY2_SWIM_DATA_LENGTH))
        {
            /*! Invalid number of parameters readout */
            rslt = BHY2_E_INVALID_EVENT_SIZE;
        }
        else
        {
            version->improvement = buffer[0];
            version->platform = buffer[1];
            version->bugfix = buffer[2];
        }
    }

    return rslt;
}

/*!
 * @brief Parsing the fifo data to SWIM output structure format
 *
 * @param[in] Swim data
 * @param[out] bhy2_swim_aglo_output to store parameter data
 *
 * @return  API error codes
 *
 */
int8_t bhy2_swim_parse_data(const uint8_t *data, struct bhy2_swim_algo_output* output)
{
    int8_t rslt = BHY2_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        output->total_distance = BHY2_LE2U16(data);
        output->length_count = BHY2_LE2U16(data + 2);
        output->lengths_freestyle = BHY2_LE2U16(data + 4);
        output->lengths_breaststroke = BHY2_LE2U16(data + 6);
        output->lengths_butterfly = BHY2_LE2U16(data + 8);
        output->lengths_backstroke = BHY2_LE2U16(data + 10);
        output->stroke_count = BHY2_LE2U16(data + 12);
    }

    return rslt;
}
