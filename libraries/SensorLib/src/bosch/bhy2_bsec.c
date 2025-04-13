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
 * @file       bhy2_bsec.c
 * @date       2023-03-24
 * @version    v1.6.0
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdlib.h>

/*********************************************************************/
/* BHY2 SensorAPI header files */
/*********************************************************************/
#include "bhy2.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhy2_bsec.h"

void bhy2_bsec_parse_air_quality(const uint8_t *payload, struct bhy2_bsec_air_quality *data)
{
    uint8_t i = 0;

    if ((payload != NULL) && (data != NULL))
    {
        data->comp_temp = BHY2_LE2U32(payload + i);
        i += 4;
        data->comp_hum = BHY2_LE2U32(payload + i);
        i += 4;
        data->comp_gas = BHY2_LE2U32(payload + i);
        i += 4;
        data->iaq = BHY2_LE2U32(payload + i);
        i += 4;
        data->static_iaq = BHY2_LE2U32(payload + i);
        i += 4;
        data->e_co2 = BHY2_LE2U32(payload + i);
        i += 4;
        data->voc = BHY2_LE2U32(payload + i);
        i += 4;
        data->iaq_accuracy = payload[i];
    }
}
