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
* @file       bhy2.c
* @date       2023-03-24
* @version    v1.6.0
*
*/

#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_defs.h"

typedef enum
{
    BHY2_BUFFER_STATUS_OK,
    BHY2_BUFFER_STATUS_RELOAD
} buffer_status_t;

union bhy2_float_u32
{
    bhy2_float as_float;
    uint32_t reg;
};

/*
 * Array base address "0" corresponds to virtual sensor id 245
 * Debug Data: 250 : event size : 18
 * 245 & 251 : Timestamp small Delta : event size : 2
 * 252 & 246 : Timestamp Large Delta : event size : 3
 * 253 & 247 : Full timestamp: event size: 6
 * 254 & 248 : Meta event: event size: 4
 * Filler: 255 : event size : 1
 */
static const uint8_t bhy2_sysid_event_size[11] = { 2, 3, 6, 4, 0, 18, 2, 3, 6, 4, 1 };

static int8_t parse_fifo(enum bhy2_fifo_type source, struct bhy2_fifo_buffer *fifo_p, struct bhy2_dev *dev);
static int8_t get_buffer_status(const struct bhy2_fifo_buffer *fifo_p, uint8_t event_size, buffer_status_t *status);
static int8_t get_time_stamp(enum bhy2_fifo_type source, uint64_t **time_stamp, struct bhy2_dev *dev);
static int8_t get_callback_info(uint8_t sensor_id, struct bhy2_fifo_parse_callback_table *info, struct bhy2_dev *dev);

/*function to check the return values in parse_fifo function*/
static inline int8_t check_return_value(int8_t result)
{

    if (result != BHY2_OK)
    {
        return result;
    }

    return BHY2_OK;
}

int8_t bhy2_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_regs(reg_addr, reg_data, length, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t length, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_set_regs(reg_addr, reg_data, length, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

static int8_t bhy2_get_and_process_fifo_support(uint8_t *int_status,
                                                int8_t *rslt,
                                                struct bhy2_fifo_buffer *fifo_temp,
                                                struct bhy2_dev *dev)
{
    uint32_t bytes_read = 0;
    int8_t temp_rslt = BHY2_OK;

    while ((*int_status || fifo_temp->remain_length) && (*rslt == BHY2_OK))
    {
        if (((BHY2_IS_INT_FIFO_W(*int_status)) == BHY2_IST_FIFO_W_DRDY) ||
            ((BHY2_IS_INT_FIFO_W(*int_status)) == BHY2_IST_FIFO_W_LTCY) ||
            ((BHY2_IS_INT_FIFO_W(*int_status)) == BHY2_IST_FIFO_W_WM) || (fifo_temp->remain_length))
        {

            /* Reset read_pos to the start of the buffer */
            fifo_temp->read_pos = 0;

            /* Append data into the work_buffer linked through fifos */
            *rslt =
                bhy2_hif_get_wakeup_fifo(&fifo_temp->buffer[fifo_temp->read_length],
                                         (fifo_temp->buffer_size - fifo_temp->read_length),
                                         &bytes_read,
                                         &fifo_temp->remain_length,
                                         &dev->hif);
            if (*rslt != BHY2_OK)
            {
                return *rslt;
            }

            fifo_temp->read_length += bytes_read;
        }

        *rslt = parse_fifo(BHY2_FIFO_TYPE_WAKEUP, fifo_temp, dev);
        *int_status = 0;
    }

    return temp_rslt;

}

int8_t bhy2_get_and_process_fifo(uint8_t *work_buffer, uint32_t buffer_size, struct bhy2_dev *dev)
{
    uint8_t int_status, int_status_bak;
    uint32_t bytes_read = 0;
    int8_t rslt;
    struct bhy2_fifo_buffer fifos;

    if ((dev == NULL) || (work_buffer == NULL))
    {
        return BHY2_E_NULL_PTR;
    }

    if (buffer_size == 0)
    {
        return BHY2_E_BUFFER;
    }

    memset(work_buffer, 0, buffer_size);
    memset(&fifos, 0, sizeof(struct bhy2_fifo_buffer));

    fifos.buffer = work_buffer;
    fifos.buffer_size = buffer_size;

    rslt = bhy2_hif_get_interrupt_status(&int_status_bak, &dev->hif);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    /* Get and process the Wake up FIFO */
    fifos.read_length = 0;
    int_status = int_status_bak;

    rslt = bhy2_get_and_process_fifo_support(&int_status, &rslt, &fifos, dev);
    if (rslt != BHY2_OK)
    {
        return rslt;
    }

    /* Get and process the Non Wake-up FIFO */
    fifos.read_length = 0;
    int_status = int_status_bak;
    while ((int_status || fifos.remain_length) && (rslt == BHY2_OK))
    {
        if (((BHY2_IS_INT_FIFO_NW(int_status)) == BHY2_IST_FIFO_NW_DRDY) ||
            ((BHY2_IS_INT_FIFO_NW(int_status)) == BHY2_IST_FIFO_NW_LTCY) ||
            ((BHY2_IS_INT_FIFO_NW(int_status)) == BHY2_IST_FIFO_NW_WM) || (fifos.remain_length))
        {
            /* Reset read_pos to the start of the buffer */
            fifos.read_pos = 0;

            /* Append data into the work_buffer linked through fifos */
            rslt = bhy2_hif_get_nonwakeup_fifo(&fifos.buffer[fifos.read_length],
                                               (fifos.buffer_size - fifos.read_length),
                                               &bytes_read,
                                               &fifos.remain_length,
                                               &dev->hif);
            if (rslt != BHY2_OK)
            {
                return rslt;
            }

            fifos.read_length += bytes_read;
        }

        rslt = parse_fifo(BHY2_FIFO_TYPE_NON_WAKEUP, &fifos, dev);
        int_status = 0;
    }

    /* Get and process the Status fifo */
    fifos.read_length = 0;
    int_status = int_status_bak;
    while ((int_status || fifos.remain_length) && (rslt == BHY2_OK))
    {
        if ((((BHY2_IS_INT_ASYNC_STATUS(int_status)) == BHY2_IST_MASK_DEBUG) || (fifos.remain_length)))
        {
            fifos.read_pos = 0;
            rslt =
                bhy2_hif_get_status_fifo_async(&fifos.buffer[fifos.read_length],
                                               (fifos.buffer_size - fifos.read_length),
                                               &bytes_read,
                                               &fifos.remain_length,
                                               &dev->hif);
            if (rslt != BHY2_OK)
            {
                return rslt;
            }

            fifos.read_length += bytes_read;
        }

        rslt = parse_fifo(BHY2_FIFO_TYPE_STATUS, &fifos, dev);
        int_status = 0;
    }

    return rslt;
}

int8_t bhy2_get_virt_sensor_cfg(uint8_t sensor_id, struct bhy2_virt_sensor_conf *virt_sensor_conf, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    union bhy2_float_u32 sample_rate;
    uint8_t buffer[12]; /* Size of parameter to get sensor config */

    if ((dev != NULL) && (virt_sensor_conf != NULL))
    {
        rslt = bhy2_hif_get_parameter((uint16_t)(BHY2_PARAM_SENSOR_CONF_0 + sensor_id),
                                      buffer,
                                      12,
                                      &bytes_read,
                                      &dev->hif);
        sample_rate.reg = BHY2_LE2U32(buffer);
        virt_sensor_conf->sample_rate = sample_rate.as_float;
        virt_sensor_conf->latency = BHY2_LE2U32(&buffer[4]);
        virt_sensor_conf->sensitivity = BHY2_LE2U16(&buffer[8]);
        virt_sensor_conf->range = BHY2_LE2U16(&buffer[10]);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_set_virt_sensor_cfg(uint8_t sensor_id, bhy2_float sample_rate, uint32_t latency, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_exec_sensor_conf_cmd(sensor_id, sample_rate, latency, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_set_virt_sensor_range(uint8_t sensor_id, uint16_t range, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer[4] = { 0 };

    if (dev != NULL)
    {
        buffer[0] = sensor_id;
        buffer[1] = (uint8_t)(range & 0xFF);
        buffer[2] = (uint8_t)((range >> 8) & 0xFF);
        buffer[3] = 0x00; /* Reserved */
        rslt = bhy2_hif_exec_cmd(BHY2_CMD_CHANGE_RANGE, buffer, 4, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_fifo_ctrl(uint32_t *fifo_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[16] = { 0 };

    if ((dev == NULL) || (fifo_ctrl == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, 16, &bytes_read, &dev->hif);

        fifo_ctrl[0] = BHY2_LE2U32(buffer); /* Wake up FIFO watermark */
        fifo_ctrl[1] = BHY2_LE2U32(&buffer[4]); /* Wake up FIFO size */
        fifo_ctrl[2] = BHY2_LE2U32(&buffer[8]); /* Non wake up FIFO watermark */
        fifo_ctrl[3] = BHY2_LE2U32(&buffer[12]); /* Non wake up FIFO size */
    }

    return rslt;
}

int8_t bhy2_set_fifo_wmark_wkup(uint32_t watermark, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };
    uint32_t read_watermark = 0;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        buffer[0] = (uint8_t)((watermark & 0xFF));
        buffer[1] = (uint8_t)(((watermark >> 8) & 0xFF));
        buffer[2] = (uint8_t)(((watermark >> 16) & 0xFF));
        buffer[3] = (uint8_t)(((watermark >> 24) & 0xFF));
        rslt = bhy2_hif_set_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &dev->hif);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        read_watermark = BHY2_LE2U32(buffer);
        if (read_watermark != watermark)
        {
            rslt = BHY2_E_PARAM_NOT_SET;
        }
    }

    return rslt;
}

int8_t bhy2_get_fifo_wmark_wkup(uint32_t *watermark, struct bhy2_dev *dev)
{
    int8_t rslt = 0;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };

    if ((dev == NULL) || (watermark == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        *watermark = BHY2_LE2U32(buffer);
    }

    return rslt;
}

int8_t bhy2_set_fifo_wmark_nonwkup(uint32_t watermark, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };
    uint32_t read_watermark = 0;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        if (rslt < 0)
        {
            return rslt;
        }

        buffer[8] = (uint8_t)((watermark & 0xFF));
        buffer[9] = (uint8_t)(((watermark >> 8) & 0xFF));
        buffer[10] = (uint8_t)(((watermark >> 16) & 0xFF));
        buffer[11] = (uint8_t)(((watermark >> 24) & 0xFF));
        rslt = bhy2_hif_set_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &dev->hif);
        if (rslt < 0)
        {
            return rslt;
        }

        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        if (rslt < 0)
        {
            return rslt;
        }

        read_watermark = BHY2_LE2U32(&buffer[8]);
        if (read_watermark != watermark)
        {
            rslt = BHY2_E_PARAM_NOT_SET;
        }
    }

    return rslt;
}

int8_t bhy2_get_fifo_wmark_nonwkup(uint32_t *watermark, struct bhy2_dev *dev)
{
    int8_t rslt = 0;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };

    if ((dev == NULL) || (watermark == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHY2_PARAM_FIFO_CTRL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        *watermark = BHY2_LE2U32(&buffer[8]);
    }

    return rslt;
}

int8_t bhy2_get_product_id(uint8_t *product_id, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_product_id(product_id, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_chip_id(uint8_t *chip_id, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_chip_id(chip_id, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_revision_id(uint8_t *revision_id, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev != NULL) && (revision_id != NULL))
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_REVISION_ID, revision_id, 1, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_rom_version(uint16_t *rom_version, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev != NULL) && (rom_version != NULL))
    {
        rslt = bhy2_hif_get_rom_version(rom_version, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_kernel_version(uint16_t *kernel_version, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_kernel_version(kernel_version, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_user_version(uint16_t *user_version, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_user_version(user_version, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_boot_status(uint8_t *boot_status, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_boot_status(boot_status, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_host_status(uint8_t *host_status, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_host_status(host_status, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_feature_status(uint8_t *feat_status, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_get_feature_status(feat_status, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_get_virt_sensor_list(struct bhy2_dev *dev)
{
    uint32_t bytes_read = 0;
    int8_t rslt = 0;

    if (dev != NULL)
    {
        rslt =
            bhy2_hif_get_parameter(BHY2_PARAM_SYS_VIRT_SENSOR_PRESENT, dev->present_buff, 32, &bytes_read, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_upload_firmware_to_ram(const uint8_t *firmware, uint32_t length, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (firmware == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_upload_firmware_to_ram(firmware, length, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_upload_firmware_to_ram_partly(const uint8_t *firmware,
                                          uint32_t total_size,
                                          uint32_t cur_pos,
                                          uint32_t packet_len,
                                          struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (firmware == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_upload_firmware_to_ram_partly(firmware, total_size, cur_pos, packet_len, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_boot_from_ram(struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_boot_program_ram(&dev->hif);
    }

    return rslt;
}

int8_t bhy2_erase_flash(uint32_t start_address, uint32_t end_addr, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_erase_flash(start_address, end_addr, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_upload_firmware_to_flash(const uint8_t *firmware, uint32_t length, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer[20] = { 0 };
    uint32_t bytes_read = 0;

    if ((dev == NULL) || (firmware == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_upload_to_flash(firmware, length, buffer, sizeof(buffer), &bytes_read, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_upload_firmware_to_flash_partly(const uint8_t *firmware,
                                            uint32_t cur_pos,
                                            uint32_t packet_len,
                                            struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (firmware == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_upload_to_flash_partly(firmware, cur_pos, packet_len, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_boot_from_flash(struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_boot_from_flash(&dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_host_interrupt_ctrl(uint8_t hintr_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERRUPT_CTRL, &hintr_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_host_interrupt_ctrl(uint8_t *hintr_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERRUPT_CTRL, hintr_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_interrupt_status(uint8_t *int_status, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_interrupt_status(int_status, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_host_intf_ctrl(uint8_t hintf_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &hintf_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_host_intf_ctrl(uint8_t *hintf_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, hintf_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_fifo_format_ctrl(uint8_t param, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer[4] = { 0 };

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        buffer[0] = param & BHY2_FIFO_FORMAT_CTRL_MASK;
        rslt = bhy2_hif_exec_cmd(BHY2_CMD_FIFO_FORMAT_CTRL, buffer, 4, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_timestamp_event_req(uint8_t ts_ev_req, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        if (ts_ev_req)
        {
            buffer = 0x01;
        }
        else
        {
            buffer = 0x00;
        }

        rslt = bhy2_hif_set_regs(BHY2_REG_TIME_EV_REQ, &buffer, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_hw_timestamp_ns(uint64_t *timestamp_ns, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (timestamp_ns == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_hw_timestamp(timestamp_ns, &dev->hif);
        *timestamp_ns *= UINT64_C(15625);
    }

    return rslt;
}

int8_t bhy2_set_host_ctrl(uint8_t host_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_regs(BHY2_REG_HOST_CTRL, &host_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_host_ctrl(uint8_t *host_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (host_ctrl == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_CTRL, host_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_soft_passthrough_transfer(union bhy2_soft_passthrough_conf *conf,
                                      uint8_t reg_addr,
                                      uint8_t length,
                                      uint8_t *reg_data,
                                      struct bhy2_dev *dev)
{
    uint32_t bytes_read = 0;
    int8_t rslt = BHY2_OK;
    uint8_t cmd[BHY2_COMMAND_PACKET_LEN] = { 0 };
    uint8_t tmp_rd_buf[BHY2_SPASS_WRITE_RESP_PACKET_LEN] = { 0 };
    uint16_t buffer_size = 0;

    if ((dev == NULL) || (conf == NULL) || (reg_data == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        conf->conf.trans_count = length;
        conf->conf.reg = reg_addr;
        if (conf->conf.direction == BHY2_SPASS_READ)
        {
            if ((BHY2_SPASS_READ_PACKET_LEN + length) % 4)
            {
                buffer_size = (uint16_t)(((BHY2_SPASS_READ_PACKET_LEN + length) / 4 + 1) * 4);
            }
            else
            {
                buffer_size = (uint16_t)(BHY2_SPASS_READ_PACKET_LEN + length);
            }

            if (buffer_size <= BHY2_COMMAND_PACKET_LEN)
            {
                rslt = bhy2_hif_exec_soft_passthrough(conf->data, 8, cmd, buffer_size, &bytes_read, &dev->hif);
                memcpy(reg_data, &cmd[9], length);
            }
            else
            {
                rslt = BHY2_E_BUFFER;
            }
        }
        else
        {
            if ((8 + length) % 4)
            {
                buffer_size = (uint16_t)(((8 + length) / 4 + 1) * 4);
            }
            else
            {
                buffer_size = (uint16_t)(8 + length);
            }

            if (buffer_size <= BHY2_COMMAND_PACKET_LEN)
            {
                memcpy(&cmd[0], conf->data, 8);
                memcpy(&cmd[8], reg_data, length);
                rslt = bhy2_hif_exec_soft_passthrough(&cmd[0],
                                                      buffer_size,
                                                      tmp_rd_buf,
                                                      sizeof(tmp_rd_buf),
                                                      &bytes_read,
                                                      &dev->hif);
            }
            else
            {
                rslt = BHY2_E_BUFFER;
            }
        }
    }

    return rslt;
}

int8_t bhy2_flush_fifo(uint8_t sensor_id, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_fifo_flush(sensor_id, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_soft_reset(struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_reset(&dev->hif);
    }

    return rslt;
}

int8_t bhy2_perform_self_test(uint8_t phys_sensor_id, struct bhy2_self_test_resp *self_test_resp, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (self_test_resp == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_do_self_test(phys_sensor_id, self_test_resp, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_perform_foc(uint8_t phys_sensor_id, struct bhy2_foc_resp *foc_status, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (foc_status == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_do_foc(phys_sensor_id, foc_status, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_orientation_matrix(uint8_t phys_sensor_id, struct bhy2_orient_matrix orient_matrix,
                                   struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_orientation_matrix(phys_sensor_id, orient_matrix, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_phys_sensor_info(uint8_t phys_sensor_id,
                                 struct bhy2_phys_sensor_info *phy_sen_info,
                                 struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (phy_sen_info == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_phys_sensor_info(phys_sensor_id, phy_sen_info, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_orientation_matrix(uint8_t phys_sensor_id,
                                   struct bhy2_orient_matrix *orient_matrix,
                                   struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    struct bhy2_phys_sensor_info phy_sen_info;

    if ((dev == NULL) || (orient_matrix == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_phys_sensor_info(phys_sensor_id, &phy_sen_info, &dev->hif);

        /*lint -e702 Info 702: Shift right of signed quantity (int) */
        orient_matrix->c[0] = (int8_t)((phy_sen_info.orientation_matrix[0] & 0x0F) << 4) >> 4;
        orient_matrix->c[1] = (int8_t)(phy_sen_info.orientation_matrix[0] & 0xF0) >> 4;
        orient_matrix->c[2] = (int8_t)((phy_sen_info.orientation_matrix[1] & 0x0F) << 4) >> 4;
        orient_matrix->c[3] = (int8_t)(phy_sen_info.orientation_matrix[1] & 0xF0) >> 4;
        orient_matrix->c[4] = (int8_t)((phy_sen_info.orientation_matrix[2] & 0x0F) << 4) >> 4;
        orient_matrix->c[5] = (int8_t)(phy_sen_info.orientation_matrix[2] & 0xF0) >> 4;
        orient_matrix->c[6] = (int8_t)((phy_sen_info.orientation_matrix[3] & 0x0F) << 4) >> 4;
        orient_matrix->c[7] = (int8_t)(phy_sen_info.orientation_matrix[3] & 0xF0) >> 4;
        orient_matrix->c[8] = (int8_t)((phy_sen_info.orientation_matrix[4] & 0x0F) << 4) >> 4;

        /*lint +e702 */
    }

    return rslt;
}

int8_t bhy2_get_sic_matrix(uint8_t *sic_matrix, uint16_t matrix_len, uint32_t *actual_len, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (sic_matrix == NULL) || (actual_len == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_bsx_state(BHY2_PARAM_BSX_SIC, sic_matrix, matrix_len, actual_len, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_sic_matrix(const uint8_t *sic_matrix, uint16_t matrix_len, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (sic_matrix == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_bsx_state(BHY2_PARAM_BSX_SIC, sic_matrix, matrix_len, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_calibration_profile(uint8_t phy_sensor_id,
                                    uint8_t *calib_prof,
                                    uint16_t prof_len,
                                    uint32_t *actual_len,
                                    struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (calib_prof == NULL) || (actual_len == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_bsx_state((uint16_t)(BHY2_PARAM_BSX_CALIB_STATE_BASE | phy_sensor_id),
                                      calib_prof,
                                      prof_len,
                                      actual_len,
                                      &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_calibration_profile(uint8_t phy_sensor_id,
                                    const uint8_t *calib_prof,
                                    uint16_t prof_len,
                                    struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (calib_prof == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_set_bsx_state((uint16_t)(BHY2_PARAM_BSX_CALIB_STATE_BASE | phy_sensor_id),
                                      calib_prof,
                                      prof_len,
                                      &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_post_mortem_data(uint8_t *post_mortem, uint32_t buffer_len, uint32_t *actual_len, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint16_t crash_dump_status_code = 0;

    if ((dev == NULL) || (post_mortem == NULL) || (actual_len == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_post_mortem(&crash_dump_status_code, actual_len, post_mortem, buffer_len, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_init(enum bhy2_intf intf,
                 bhy2_read_fptr_t read,
                 bhy2_write_fptr_t write,
                 bhy2_delay_us_fptr_t delay_us,
                 uint32_t read_write_len,
                 void *intf_ptr,
                 struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        memset(dev, 0, sizeof(struct bhy2_dev));

        rslt = bhy2_hif_init(intf, read, write, delay_us, read_write_len, intf_ptr, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_register_fifo_parse_callback(uint8_t sensor_id,
                                         bhy2_fifo_parse_callback_t callback,
                                         void *callback_ref,
                                         struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t i = 0;

    if ((dev == NULL) || (callback == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        for (i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
        {
            if (dev->table[i].sensor_id == 0)
            {
                dev->table[i].sensor_id = sensor_id;
                dev->table[i].callback = callback;
                dev->table[i].callback_ref = callback_ref;
                break;
            }
        }

        if (i == BHY2_MAX_SIMUL_SENSORS)
        {
            rslt = BHy2_E_INSUFFICIENT_MAX_SIMUL_SENSORS;
        }
    }

    return rslt;
}

int8_t bhy2_deregister_fifo_parse_callback(uint8_t sensor_id, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t i = 0;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        for (i = 0; i < BHY2_MAX_SIMUL_SENSORS; i++)
        {
            if (dev->table[i].sensor_id == sensor_id)
            {
                dev->table[i].sensor_id = 0;
                dev->table[i].callback = NULL;
                dev->table[i].callback_ref = NULL;
                break;
            }
        }
    }

    return rslt;
}

int8_t bhy2_update_virtual_sensor_list(struct bhy2_dev *dev)
{
    int8_t rslt;
    uint8_t sensor_id;
    struct bhy2_sensor_info info;
    uint8_t sensor_index;
    uint8_t bit_mask;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /* Each bit corresponds to presence of Virtual sensor */
        rslt = bhy2_get_virt_sensor_list(dev);
        if (rslt == BHY2_OK)
        {
            /* Padding: Sensor id*/
            dev->event_size[0] = 1;

            for (sensor_id = 1; (sensor_id < BHY2_SPECIAL_SENSOR_ID_OFFSET) && (rslt == BHY2_OK); sensor_id++)
            {
                sensor_index = (uint8_t)(sensor_id / 8);
                bit_mask = (uint8_t)(0x01 << (sensor_id % 8));

                if (dev->present_buff[sensor_index] & bit_mask)
                {
                    rslt = bhy2_hif_get_sensor_info(sensor_id, &info, &dev->hif);
                    if (rslt == BHY2_OK)
                    {
                        if (info.event_size == 0)
                        {
                            rslt = BHY2_E_INVALID_EVENT_SIZE;
                            break;
                        }
                        else
                        {
                            dev->event_size[sensor_id] = info.event_size;
                        }
                    }
                }
            }

            for (sensor_id = BHY2_N_VIRTUAL_SENSOR_MAX - 1; sensor_id >= BHY2_SPECIAL_SENSOR_ID_OFFSET; sensor_id--)
            {
                dev->event_size[sensor_id] = bhy2_sysid_event_size[sensor_id - BHY2_SPECIAL_SENSOR_ID_OFFSET];
            }
        }
    }

    return rslt;
}

int8_t bhy2_get_sensor_info(uint8_t sensor_id, struct bhy2_sensor_info *info, struct bhy2_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (info == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_sensor_info(sensor_id, info, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_set_parameter(uint16_t param, const uint8_t *buffer, uint32_t length, struct bhy2_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (buffer == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_exec_cmd(param, buffer, length, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_parameter(uint16_t param, uint8_t *buffer, uint32_t length, uint32_t *actual_len, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (buffer == NULL) || (actual_len == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(param, buffer, length, actual_len, &dev->hif);
    }

    return rslt;
}

int8_t bhy2_get_error_value(uint8_t *error_value, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev == NULL) || (error_value == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_get_regs(BHY2_REG_ERROR_VALUE, error_value, 1, dev);
    }

    return rslt;
}

int8_t bhy2_set_data_injection_mode(enum bhy2_data_inj_mode mode, struct bhy2_dev *dev)
{
    uint8_t data_inj_payload[8] = { 0 };
    int8_t rslt = BHY2_OK;
    uint8_t work_buffer[256] = { 0 };
    uint32_t actual_len = 0;
    uint16_t cmd = BHY2_CMD_SET_INJECT_MODE;

    data_inj_payload[0] = (uint8_t)(cmd & 0xFF);
    data_inj_payload[1] = (uint8_t)((cmd >> 8) & 0xFF);
    data_inj_payload[2] = BHY2_DATA_INJECT_MODE_PAYLOAD_LEN;

    if (dev != NULL)
    {

        switch (mode)
        {
            case BHY2_NORMAL_MODE:
                data_inj_payload[4] = 0;
                break;
            case BHY2_REAL_TIME_INJECTION:
                data_inj_payload[4] = 1;
                break;
            case BHY2_STEP_BY_STEP_INJECTION:
                data_inj_payload[4] = 2;
                break;
            default:
                rslt = BHY2_E_INVALID_PARAM;
        }

        if (rslt == BHY2_OK)
        {
            rslt = bhy2_hif_set_inject_data_mode(data_inj_payload,
                                                 sizeof (data_inj_payload),
                                                 work_buffer,
                                                 256,
                                                 &actual_len,
                                                 &dev->hif);
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhy2_dev *dev)
{
    return bhy2_hif_inject_data(payload, payload_len, &dev->hif);
}

uint8_t bhy2_is_sensor_available(uint8_t sensor_id, const struct bhy2_dev *dev)
{
    uint8_t offset = sensor_id / 8;
    uint8_t mask = 1U << (sensor_id & 0x7);
    uint8_t sensor_present = 0;

    if (dev != NULL)
    {
        sensor_present = (dev->present_buff[offset] & mask);
    }

    return sensor_present;
}

/**
 * @brief Function to get the BHy260 variant ID
 */
int8_t bhy2_get_variant_id(uint32_t *variant_id, struct bhy2_dev *dev)
{
    int8_t rslt;
    uint8_t cfg_read_command[] = { 0x16, 0x00, 0x00, 0x00 };
    uint8_t buffer[118] = { 0 };

    rslt = bhy2_soft_reset(dev);
    if (rslt == BHY2_OK)
    {
        rslt = bhy2_set_regs(BHY2_REG_CHAN_CMD, cfg_read_command, sizeof(cfg_read_command), dev);

        /* Give the device some time to prepare data */
        dev->hif.delay_us(10000, NULL);
    }

    if (rslt == BHY2_OK)
    {
        /* Read status FIFO. First check if response is correct */
        rslt = bhy2_get_regs(BHY2_REG_CHAN_STATUS, buffer, 4, dev);
    }

    if (rslt == BHY2_OK)
    {
        if (!((buffer[0] == 0x10) && (buffer[1] == 0x00) && (buffer[2] == sizeof(buffer)) && (buffer[3] == 0)))
        {
            rslt = BHY2_E_INVALID_PARAM;
        }
    }

    if (rslt == BHY2_OK)
    {
        /* Read out the configuration */
        rslt = bhy2_get_regs(BHY2_REG_CHAN_STATUS, buffer, sizeof(buffer), dev);
        if (rslt == BHY2_OK)
        {
            *variant_id = ((uint32_t)buffer[40] << 24) | ((uint32_t)buffer[41] << 16) | ((uint32_t)buffer[42] << 8) |
                          buffer[43];
        }
    }

    return rslt;
}

static int8_t get_callback_info(uint8_t sensor_id, struct bhy2_fifo_parse_callback_table *info, struct bhy2_dev *dev)
{

    int8_t rslt = BHY2_OK;
    uint8_t i = 0, j;

    if ((dev != NULL) && (info != NULL))
    {
        for (j = 0; j < BHY2_MAX_SIMUL_SENSORS; j++)
        {
            if (sensor_id == dev->table[j].sensor_id)
            {
                *info = dev->table[j];
                i = j;
                break;
            }
        }

        if (i == BHY2_MAX_SIMUL_SENSORS)
        {
            rslt = BHy2_E_INSUFFICIENT_MAX_SIMUL_SENSORS;
        }
        else
        {
            if ((sensor_id >= BHY2_SPECIAL_SENSOR_ID_OFFSET) && (dev->event_size[sensor_id] == 0))
            {
                dev->event_size[sensor_id] = bhy2_sysid_event_size[sensor_id - BHY2_SPECIAL_SENSOR_ID_OFFSET];
            }

            if ((sensor_id == 0) && (dev->event_size[sensor_id] == 0))
            {
                dev->event_size[sensor_id] = 1;
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

static int8_t get_buffer_status(const struct bhy2_fifo_buffer *fifo_p, uint8_t event_size, buffer_status_t *status)
{
    if ((fifo_p->read_pos + event_size) <= fifo_p->read_length)
    {
        *status = BHY2_BUFFER_STATUS_OK;
    }
    else
    {
        *status = BHY2_BUFFER_STATUS_RELOAD;
    }

    return BHY2_OK;
}

static int8_t get_time_stamp(enum bhy2_fifo_type source, uint64_t **time_stamp, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (source < BHY2_FIFO_TYPE_MAX)
    {
        *time_stamp = &dev->last_time_stamp[source];
    }
    else
    {
        rslt = BHY2_E_INVALID_FIFO_TYPE;
    }

    return rslt;
}

static int8_t parse_fifo_support(struct bhy2_fifo_buffer *fifo_buf)
{
    uint8_t i;

    if (fifo_buf->read_length)
    {
        if (fifo_buf->read_length < fifo_buf->read_pos)
        {
            return BHY2_E_INVALID_PARAM;
        }

        fifo_buf->read_length -= fifo_buf->read_pos;
        if (fifo_buf->read_length)
        {
            for (i = 0; i < fifo_buf->read_length; i++)
            {
                fifo_buf->buffer[i] = fifo_buf->buffer[fifo_buf->read_pos + i];
            }
        }
    }

    return BHY2_OK;
}

static int8_t parse_fifo(enum bhy2_fifo_type source, struct bhy2_fifo_buffer *fifo_p, struct bhy2_dev *dev)
{
    uint8_t tmp_sensor_id = 0;
    int8_t rslt = BHY2_OK;
    uint32_t tmp_read_pos;
    struct bhy2_fifo_parse_data_info data_info;
    uint64_t *time_stamp;
    struct bhy2_fifo_parse_callback_table info = { 0 };
    buffer_status_t status = BHY2_BUFFER_STATUS_OK;

    for (; (fifo_p->read_pos < fifo_p->read_length) && (status == BHY2_BUFFER_STATUS_OK);)
    {
        tmp_read_pos = fifo_p->read_pos;
        tmp_sensor_id = fifo_p->buffer[tmp_read_pos];

        rslt = get_time_stamp(source, &time_stamp, dev);
        rslt = check_return_value(rslt);
        switch (tmp_sensor_id)
        {
            case BHY2_SYS_ID_FILLER:
            case BHY2_SYS_ID_PADDING:
                fifo_p->read_pos += 1;
                break;
            case BHY2_SYS_ID_TS_SMALL_DELTA_WU:
            case BHY2_SYS_ID_TS_SMALL_DELTA:
                rslt = get_buffer_status(fifo_p, 2, &status);
                rslt = check_return_value(rslt);
                if (status != BHY2_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp += fifo_p->buffer[tmp_read_pos + 1];
                fifo_p->read_pos += 2;
                break;
            case BHY2_SYS_ID_TS_LARGE_DELTA:
            case BHY2_SYS_ID_TS_LARGE_DELTA_WU:
                rslt = get_buffer_status(fifo_p, 3, &status);
                rslt = check_return_value(rslt);

                if (status != BHY2_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp += BHY2_LE2U16(fifo_p->buffer + tmp_read_pos + 1);
                fifo_p->read_pos += 3;
                break;
            case BHY2_SYS_ID_BHY2_LOG_DOSTEP:
                rslt = get_buffer_status(fifo_p, 23, &status);
                rslt = check_return_value(rslt);

                if (status != BHY2_BUFFER_STATUS_OK)
                {
                    break;
                }

                fifo_p->read_pos += 23;

                break;
            case BHY2_SYS_ID_TS_FULL:
            case BHY2_SYS_ID_TS_FULL_WU:
                rslt = get_buffer_status(fifo_p, 6, &status);
                rslt = check_return_value(rslt);

                if (status != BHY2_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp = BHY2_LE2U40(fifo_p->buffer + tmp_read_pos + UINT8_C(1));
                fifo_p->read_pos += 6;
                break;
            default:
                rslt = get_callback_info(tmp_sensor_id, &info, dev);
                rslt = check_return_value(rslt);
                rslt = get_buffer_status(fifo_p, dev->event_size[tmp_sensor_id], &status); /*lint !e838 suppressing
                                                                                            * previously assigned value
                                                                                            * not used info */
                rslt = check_return_value(rslt);
                if (status != BHY2_BUFFER_STATUS_OK)
                {
                    break;
                }

                if (info.callback != NULL)
                {
                    /* Read position is incremented by 1 to exclude sensor id */
                    data_info.data_ptr = &fifo_p->buffer[tmp_read_pos + 1];
                    data_info.fifo_type = source;
                    data_info.time_stamp = time_stamp;
                    data_info.sensor_id = tmp_sensor_id;
                    data_info.data_size = dev->event_size[tmp_sensor_id];
                    info.callback(&data_info, info.callback_ref);
                }

                fifo_p->read_pos += dev->event_size[tmp_sensor_id];
                break;
        }
    }

    rslt = parse_fifo_support(fifo_p);

    return rslt;
}

int8_t bhy2_clear_fifo(uint8_t flush_cfg, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer[4] = { 0 };

    if (dev != NULL)
    {
        buffer[0] = flush_cfg;
        rslt = bhy2_hif_exec_cmd(BHY2_CMD_FIFO_FLUSH, buffer, 4, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_read_status(uint16_t *status_code,
                        uint8_t *status_buff,
                        uint32_t status_len,
                        uint32_t *actual_len,
                        struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev != NULL)
    {
        rslt = bhy2_hif_wait_status_ready(&dev->hif);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        rslt = bhy2_hif_get_status_fifo(status_code, status_buff, status_len, actual_len, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}
