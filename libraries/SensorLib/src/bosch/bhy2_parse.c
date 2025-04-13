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
 * @file       bhy2_parse.c
 * @date       2023-03-24
 * @version    v1.6.0
 *
 */

#include "bhy2.h"
#include "bhy2_parse.h"

void bhy2_parse_temperature_celsius(const uint8_t *data, bhy2_float *temperature)
{
    /* 1 LSB = 1/100 degC */
    float scale_factor = (float)1 / 100;

    *temperature = BHY2_LE2S16(data) * scale_factor;
}

void bhy2_parse_humidity(const uint8_t *data, bhy2_float *humidity)
{
    float scale_factor = (float)1;

    *humidity = data[0] * scale_factor;
}

void bhy2_parse_pressure(const uint8_t *data, bhy2_float *pressure)
{
    /* 1 LSB = 1/128 Pa */
    float scale_factor = (float)1 / 128;

    *pressure = (float)BHY2_LE2U24(data) * scale_factor;
}

void bhy2_parse_altitude(const uint8_t *data, bhy2_float *altitude)
{
    *altitude = (float)(data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

void bhy2_parse_quaternion(const uint8_t *data, struct bhy2_data_quaternion *quaternion)
{
    quaternion->x = BHY2_LE2S16(data);
    quaternion->y = BHY2_LE2S16(data + 2);
    quaternion->z = BHY2_LE2S16(data + 4);
    quaternion->w = BHY2_LE2S16(data + 6);
    quaternion->accuracy = BHY2_LE2U16(data + 8);
}

void bhy2_parse_xyz(const uint8_t *data, struct bhy2_data_xyz *vector)
{
    vector->x = BHY2_LE2S16(data);
    vector->y = BHY2_LE2S16(data + 2);
    vector->z = BHY2_LE2S16(data + 4);
}

void bhy2_parse_orientation(const uint8_t *data, struct bhy2_data_orientation *orientation)
{
    orientation->heading = BHY2_LE2S16(data);
    orientation->pitch = BHY2_LE2S16(data + 2);
    orientation->roll = BHY2_LE2S16(data + 4);
}

uint32_t bhy2_parse_step_counter(const uint8_t *data)
{
    return (uint32_t)(data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}
