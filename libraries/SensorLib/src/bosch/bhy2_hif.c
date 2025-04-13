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
 * @file       bhy2_hif.c
 * @date       2023-03-24
 * @version    v1.6.0
 *
 */

#include "bhy2_hif.h"

/*! Mask definitions for SPI read/write address */
#define BHY2_SPI_RD_MASK UINT8_C(0x80)
#define BHY2_SPI_WR_MASK UINT8_C(0x7F)

union bhy2_hif_float_u32
{
    bhy2_float as_float;
    uint32_t reg;
};

static void bhy2_hif_exec_cmd_generic_support(const uint16_t *cmd,
                                              uint8_t *cmd_buf,
                                              uint32_t *temp_length,
                                              uint32_t *total_length,
                                              const uint32_t *len,
                                              const uint32_t *pre_len,
                                              const uint32_t *cmd_len)
{
    *total_length = *pre_len + *len;

    if (*cmd_len)
    {
        *temp_length = *pre_len + *cmd_len;
    }
    else
    {
        *temp_length = *total_length;
    }

    /* Align 4 bytes */
    if (*temp_length % 4)
    {

        *temp_length = BHY2_ROUND_WORD_HIGHER(*temp_length);
    }

    cmd_buf[0] = (uint8_t)(*cmd & 0xFF);
    cmd_buf[1] = (uint8_t)((*cmd >> 8) & 0xFF);

    /* Length in word */
    if (*cmd == BHY2_CMD_UPLOAD_TO_PROGRAM_RAM)
    {
        cmd_buf[2] = (*temp_length / 4) & 0xFF;
        cmd_buf[3] = ((*temp_length / 4) >> 8) & 0xFF;
    }
    /* Length in byte */
    else
    {
        cmd_buf[2] = *temp_length & 0xFF;
        cmd_buf[3] = (*temp_length >> 8) & 0xFF;
    }
}

/*
 * If a command need several frames to upload to sensor,
 * this command only be used in the first frame.
 *
 * |------------------------------------------------|---------------|
 * |<--               frame one                  -->|<--frame two-->|
 * |------------------------------------------------|---------------|
 * |                  |<--pre_length-->|<--length-->|               |
 * |------------------|----------------|------------|---------------|
 * | cmd | cmd_length |   pre_payload  |   data 1   |     data 2    |
 * |-----------------------------------|------------|---------------|
 *                                     |          payload           |
 *                                     |------------|---------------|
 *                                     |<--     cmd_length       -->|
 *                                     |----------------------------|
 */
static int8_t bhy2_hif_exec_cmd_generic(uint16_t cmd,
                                        const uint8_t *payload,
                                        uint32_t length,
                                        const uint8_t *pre_payload,
                                        uint32_t pre_length,
                                        uint32_t cmd_length,
                                        struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint32_t remain, trans_len, copy_len, pos, total_len, temp_total_len, loop_remain_len, max_len;
    uint8_t command_buf[BHY2_COMMAND_PACKET_LEN];

    if ((hif != NULL) && !((length != 0) && (payload == NULL)) && !((pre_length != 0) && (pre_payload == NULL)))
    {
        if (hif->read_write_len != 0)
        {
            bhy2_hif_exec_cmd_generic_support(&cmd,
                                              command_buf,
                                              &temp_total_len,
                                              &total_len,
                                              &length,
                                              &pre_length,
                                              &cmd_length);
            pos = BHY2_COMMAND_HEADER_LEN;
            remain = total_len;
            loop_remain_len = remain + pos;
            max_len = BHY2_COMMAND_PACKET_LEN - BHY2_COMMAND_HEADER_LEN;

            if (hif->read_write_len < max_len)
            {
                max_len = hif->read_write_len;
            }

            while ((loop_remain_len > 0) && (rslt == BHY2_OK))
            {
                if ((remain + pos) > max_len)
                {
                    trans_len = max_len;
                    copy_len = max_len - pos;
                }
                else
                {
                    trans_len = remain + pos;
                    copy_len = remain;

                    /* Align to 4 bytes */
                    if (trans_len % 4)
                    {

                        trans_len = BHY2_ROUND_WORD_HIGHER(trans_len);
                    }

                    if (trans_len > (BHY2_COMMAND_PACKET_LEN - BHY2_COMMAND_HEADER_LEN))
                    {
                        trans_len = BHY2_COMMAND_PACKET_LEN - BHY2_COMMAND_HEADER_LEN;
                    }
                }

                if (copy_len > 0)
                {
                    if (remain >= (length + copy_len))
                    {
                        memcpy(&command_buf[pos], &pre_payload[total_len - remain], copy_len);
                    }
                    else if (remain > length)
                    {
                        memcpy(&command_buf[pos], &pre_payload[total_len - remain], remain - length);
                        memcpy(&command_buf[pos + remain - length], payload, copy_len - (remain - length));
                    }
                    else
                    {
                        memcpy(&command_buf[pos], &payload[length - remain], copy_len);
                    }
                }

                if ((trans_len - (pos + copy_len)) > 0)
                {
                    memset(&command_buf[pos + copy_len], 0, BHY2_COMMAND_PACKET_LEN - (pos + copy_len));
                }

                rslt = bhy2_hif_set_regs(BHY2_REG_CHAN_CMD, command_buf, trans_len, hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }

                pos = 0;
                remain -= copy_len;
                loop_remain_len = remain;
            }
        }
        else
        {
            rslt = BHY2_E_INVALID_PARAM;
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_wait_status_ready(struct bhy2_hif_dev *hif)
{
    uint16_t retry;
    uint8_t int_status;
    int8_t rslt;

    /* Wait status ready */
    for (retry = 0; retry < BHY2_QUERY_PARAM_STATUS_READY_MAX_RETRY; ++retry)
    {
        rslt = bhy2_hif_get_interrupt_status(&int_status, hif);
        if (rslt == BHY2_OK)
        {
            if (int_status & BHY2_IST_MASK_STATUS)
            {
                rslt = BHY2_OK;
                break;
            }

            /* 10ms */
            rslt = bhy2_hif_delay_us(10000, hif);
            if (rslt != BHY2_OK)
            {
                return rslt;
            }
        }
        else
        {
            return rslt; /*break; */
        }
    }

    return rslt;
}

static int8_t bhy2_hif_check_boot_status_ram(struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint16_t i;
    uint8_t boot_status;

    /* total 5s */
    for (i = 0; i < BHY2_BST_CHECK_RETRY; ++i)
    {
        /* 50ms */
        rslt = bhy2_hif_delay_us(50000, hif);
        if (rslt < 0)
        {
            return rslt;
        }

        rslt = bhy2_hif_get_regs(BHY2_REG_BOOT_STATUS, &boot_status, sizeof(boot_status), hif);
        if (rslt < 0)
        {
            return rslt;
        }

        if ((boot_status & BHY2_BST_HOST_INTERFACE_READY) && (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) &&
            (!(boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR)))
        {
            break;
        }
    }

    if (i == BHY2_BST_CHECK_RETRY)
    {
        return BHY2_E_TIMEOUT;
    }

    return BHY2_OK;
}

static int8_t bhy2_hif_check_boot_status_flash(struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint16_t i;
    uint8_t boot_status;

    /* total 5s */
    for (i = 0; i < BHY2_BST_CHECK_RETRY; ++i)
    {
        /* 50ms */
        rslt = bhy2_hif_delay_us(50000, hif);
        if (rslt < 0)
        {
            return rslt;
        }

        rslt = bhy2_hif_get_regs(BHY2_REG_BOOT_STATUS, &boot_status, sizeof(boot_status), hif);
        if (rslt < 0)
        {
            return rslt;
        }

        if ((boot_status & BHY2_BST_HOST_INTERFACE_READY) && (boot_status & BHY2_BST_FLASH_VERIFY_DONE) &&
            (!(boot_status & BHY2_BST_FLASH_VERIFY_ERROR)))
        {
            break;
        }
    }

    if (i == BHY2_BST_CHECK_RETRY)
    {
        return BHY2_E_TIMEOUT;
    }

    return BHY2_OK;
}

static int8_t bhy2_hif_get_fifo(uint8_t reg,
                                uint8_t *fifo,
                                uint32_t fifo_len,
                                uint32_t *bytes_read,
                                uint32_t *bytes_remain,
                                struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint8_t n_bytes[2];
    uint32_t read_len;
    uint32_t read_write_len;
    uint32_t offset;

    if ((hif != NULL) && (fifo != NULL) && (bytes_read != NULL) && (bytes_remain != NULL))
    {
        read_write_len = hif->read_write_len;

        if (*bytes_remain == 0)
        {
            rslt = bhy2_hif_get_regs(reg, n_bytes, 2, hif); /* Read the the available size */
            *bytes_remain = BHY2_LE2U16(n_bytes);
        }

        if ((*bytes_remain != 0) && (rslt == BHY2_OK))
        {
            if (fifo_len < *bytes_remain)
            {
                *bytes_read = fifo_len;
            }
            else
            {
                *bytes_read = *bytes_remain;
            }

            read_len = *bytes_read;
            offset = 0;
            while (read_len > read_write_len)
            {
                rslt = bhy2_hif_get_regs(reg, &fifo[offset], read_write_len, hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }

                read_len -= read_write_len;
                offset += read_write_len;
            }

            if (read_len != 0)
            {
                rslt = bhy2_hif_get_regs(reg, &fifo[offset], read_len, hif);
            }

            *bytes_remain -= *bytes_read;
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_init(enum bhy2_intf intf,
                     bhy2_read_fptr_t read,
                     bhy2_write_fptr_t write,
                     bhy2_delay_us_fptr_t delay_us,
                     uint32_t read_write_len,
                     void *intf_ptr,
                     struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;

    if ((hif != NULL) && (read != NULL) && (write != NULL) && (delay_us != NULL))
    {
        hif->read = read;
        hif->write = write;
        hif->delay_us = delay_us;
        hif->intf_ptr = intf_ptr;
        hif->intf = intf;
        if (read_write_len % 4)
        {

            hif->read_write_len = BHY2_ROUND_WORD_LOWER(read_write_len);
        }
        else
        {
            hif->read_write_len = read_write_len;
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;

    if ((hif != NULL) && (hif->read != NULL) && (reg_data != NULL))
    {
        if (hif->intf == BHY2_SPI_INTERFACE)
        {
            reg_addr |= BHY2_SPI_RD_MASK;
        }

        hif->intf_rslt = hif->read(reg_addr, reg_data, length, hif->intf_ptr);
        if (hif->intf_rslt != BHY2_INTF_RET_SUCCESS)
        {
            rslt = BHY2_E_IO;
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;

    if ((hif != NULL) && (hif->write != NULL) && (reg_data != NULL))
    {
        if (hif->intf == BHY2_SPI_INTERFACE)
        {
            reg_addr &= BHY2_SPI_WR_MASK;
        }

        hif->intf_rslt = hif->write(reg_addr, reg_data, length, hif->intf_ptr);
        if (hif->intf_rslt != BHY2_INTF_RET_SUCCESS)
        {
            rslt = BHY2_E_IO;
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_delay_us(uint32_t period_us, const struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;

    if ((hif != NULL) && (hif->delay_us != NULL))
    {
        hif->delay_us(period_us, hif->intf_ptr);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_exec_cmd(uint16_t cmd, const uint8_t *payload, uint32_t length, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_exec_cmd_generic(cmd, payload, length, NULL, 0, 0, hif);
}

int8_t bhy2_hif_get_parameter(uint16_t param,
                              uint8_t *payload,
                              uint32_t payload_len,
                              uint32_t *actual_len,
                              struct bhy2_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t prev_hif_ctrl, hif_ctrl;
    int8_t rslt = BHY2_OK;

    if ((hif != NULL) && (payload != NULL) && (actual_len != NULL))
    {
        *actual_len = 0;

        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &hif_ctrl, 1, hif);
        if (rslt == BHY2_OK)
        {
            prev_hif_ctrl = hif_ctrl;
            hif_ctrl &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL)); /* Change the Status FIFO to synchronous mode
                                                                           * */
            if (hif_ctrl != prev_hif_ctrl)
            {
                rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &hif_ctrl, 1, hif);
            }

            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_exec_cmd(param | BHY2_PARAM_READ_MASK, NULL, 0, hif);
                if (rslt == BHY2_OK)
                {
                    rslt = bhy2_hif_wait_status_ready(hif);
                    if (rslt == BHY2_OK)
                    {
                        rslt = bhy2_hif_get_status_fifo(&code, payload, payload_len, actual_len, hif);

                        if (rslt == BHY2_OK)
                        {

                            if (hif_ctrl != prev_hif_ctrl)
                            {
                                hif_ctrl = prev_hif_ctrl;
                                rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &hif_ctrl, 1, hif);
                            }

                            if (rslt == BHY2_OK)
                            {
                                if (code != param)
                                {
                                    rslt = BHY2_E_TIMEOUT;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_set_parameter(uint16_t param, const uint8_t *payload, uint32_t length, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_exec_cmd(param, payload, length, hif);
}

int8_t bhy2_hif_get_product_id(uint8_t *product_id, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_PRODUCT_ID, product_id, 1, hif);
}

int8_t bhy2_hif_get_chip_id(uint8_t *chip_id, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_CHIP_ID, chip_id, 1, hif);
}

int8_t bhy2_hif_get_rom_version(uint16_t *rom_version, struct bhy2_hif_dev *hif)
{
    uint8_t buffer[2];
    int8_t rslt;

    if (rom_version != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_ROM_VERSION_0, buffer, 2, hif);
        *rom_version = BHY2_LE2U16(buffer);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_kernel_version(uint16_t *kernel_version, struct bhy2_hif_dev *hif)
{
    uint8_t buffer[2];
    int8_t rslt;

    if (kernel_version != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_KERNEL_VERSION_0, buffer, 2, hif);
        *kernel_version = BHY2_LE2U16(buffer);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_user_version(uint16_t *user_version, struct bhy2_hif_dev *hif)
{
    uint8_t buffer[2];
    int8_t rslt;

    if (user_version != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_USER_VERSION_0, buffer, 2, hif);
        *user_version = BHY2_LE2U16(buffer);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_boot_status(uint8_t *boot_status, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_BOOT_STATUS, boot_status, 1, hif);
}

int8_t bhy2_hif_get_host_status(uint8_t *host_status, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_HOST_STATUS, host_status, 1, hif);
}

int8_t bhy2_hif_get_feature_status(uint8_t *feat_status, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_FEATURE_STATUS, feat_status, 1, hif);
}

int8_t bhy2_hif_get_interrupt_status(uint8_t *int_status, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_INT_STATUS, int_status, 1, hif);
}

int8_t bhy2_hif_get_fw_error(uint8_t *fw_error, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_regs(BHY2_REG_ERROR_VALUE, fw_error, 1, hif);
}

int8_t bhy2_hif_reset(struct bhy2_hif_dev *hif)
{
    uint8_t reset_req = BHY2_REQUEST_RESET;
    uint8_t boot_status = 0;
    int8_t rslt;

    /* Timeout at 150ms (15 * 10000 microseconds) */
    uint16_t count = 15;

    rslt = bhy2_hif_set_regs(BHY2_REG_RESET_REQ, &reset_req, 1, hif);
    if (rslt == BHY2_OK)
    {
        while (count--)
        {
            rslt = bhy2_hif_delay_us(10000, hif);
            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_get_regs(BHY2_REG_BOOT_STATUS, &boot_status, 1, hif);
                if (rslt == BHY2_OK)
                {
                    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
                    {
                        break;
                    }
                }
            }
        }

        if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY))
        {
            rslt = BHY2_E_TIMEOUT;
        }
    }

    return rslt;
}

int8_t bhy2_hif_upload_firmware_to_ram(const uint8_t *firmware, uint32_t length, struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint16_t magic;

    if ((hif != NULL) && (firmware != NULL))
    {
        magic = BHY2_LE2U16(firmware);
        if (magic != BHY2_FW_MAGIC)
        {
            rslt = BHY2_E_MAGIC;
        }
        else
        {
            rslt = bhy2_hif_exec_cmd(BHY2_CMD_UPLOAD_TO_PROGRAM_RAM, firmware, length, hif);
            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_check_boot_status_ram(hif);
                
            }
           
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_upload_firmware_to_ram_partly(const uint8_t *firmware,
                                              uint32_t total_size,
                                              uint32_t cur_pos,
                                              uint32_t packet_len,
                                              struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint16_t magic;

    if ((hif != NULL) && (firmware != NULL))
    {
        if (cur_pos == 0)
        {
            magic = BHY2_LE2U16(firmware);
            if (magic != BHY2_FW_MAGIC)
            {
                rslt = BHY2_E_MAGIC;
            }

            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_exec_cmd_generic(BHY2_CMD_UPLOAD_TO_PROGRAM_RAM,
                                                 firmware,
                                                 packet_len,
                                                 NULL,
                                                 0,
                                                 total_size,
                                                 hif);
            }
        }
        else
        {
            rslt = bhy2_hif_set_regs(BHY2_REG_CHAN_CMD, firmware, packet_len, hif);
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_boot_program_ram(struct bhy2_hif_dev *hif)
{
    int8_t rslt;

    rslt = bhy2_hif_exec_cmd(BHY2_CMD_BOOT_PROGRAM_RAM, NULL, 0, hif);
    if (rslt == BHY2_OK)
    {
        rslt = bhy2_hif_check_boot_status_ram(hif);
    }

    return rslt;
}

int8_t bhy2_hif_get_wakeup_fifo(uint8_t *fifo,
                                uint32_t fifo_len,
                                uint32_t *bytes_read,
                                uint32_t *bytes_remain,
                                struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_fifo(BHY2_REG_CHAN_FIFO_W, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhy2_hif_get_nonwakeup_fifo(uint8_t *fifo,
                                   uint32_t fifo_len,
                                   uint32_t *bytes_read,
                                   uint32_t *bytes_remain,
                                   struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_fifo(BHY2_REG_CHAN_FIFO_NW, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhy2_hif_get_status_fifo(uint16_t *status_code,
                                uint8_t *fifo,
                                uint32_t fifo_len,
                                uint32_t *bytes_remain,
                                struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint8_t tmp_fifo[4];

    rslt = bhy2_hif_get_regs(BHY2_REG_CHAN_STATUS, tmp_fifo, 4, hif);
    if (rslt == BHY2_OK)
    {
        *status_code = BHY2_LE2U16(&tmp_fifo[0]);
        *bytes_remain = BHY2_LE2U16(&tmp_fifo[2]);

        if (*bytes_remain != 0)
        {
            if (fifo_len < *bytes_remain)
            {
                rslt = BHY2_E_BUFFER;
            }
            else
            {
                rslt = bhy2_hif_get_regs(BHY2_REG_CHAN_STATUS, fifo, *bytes_remain, hif);
            }
        }
    }

    return rslt;
}

int8_t bhy2_hif_get_status_fifo_async(uint8_t *fifo,
                                      uint32_t fifo_len,
                                      uint32_t *bytes_read,
                                      uint32_t *bytes_remain,
                                      struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_fifo(BHY2_REG_CHAN_STATUS, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhy2_hif_exec_sensor_conf_cmd(uint8_t sensor_id,
                                     bhy2_float sample_rate,
                                     uint32_t latency,
                                     struct bhy2_hif_dev *hif)
{
    uint8_t tmp_buf[8];
    union bhy2_hif_float_u32 sample_rate_u;

    sample_rate_u.as_float = sample_rate;

    /* Sample rate is 32bits, latency is 24bits */
    tmp_buf[0] = sensor_id;
    tmp_buf[1] = (uint8_t)(sample_rate_u.reg & 0xFF);
    tmp_buf[2] = (uint8_t)((sample_rate_u.reg >> 8) & 0xFF);
    tmp_buf[3] = (uint8_t)((sample_rate_u.reg >> 16) & 0xFF);
    tmp_buf[4] = (uint8_t)((sample_rate_u.reg >> 24) & 0xFF);
    tmp_buf[5] = (uint8_t)(latency & 0xFF);
    tmp_buf[6] = (uint8_t)((latency >> 8) & 0xFF);
    tmp_buf[7] = (uint8_t)((latency >> 16) & 0xFF);

    return bhy2_hif_exec_cmd(BHY2_CMD_CONFIG_SENSOR, tmp_buf, 8, hif);
}

int8_t bhy2_hif_set_fifo_flush(uint8_t sensor_id, struct bhy2_hif_dev *hif)
{
    uint8_t tmp_buf[4] = {sensor_id, 0, 0, 0};

    return bhy2_hif_exec_cmd(BHY2_CMD_FIFO_FLUSH, tmp_buf, 4, hif);
}

int8_t bhy2_hif_exec_soft_passthrough(const uint8_t *payload,
                                      uint32_t payload_len,
                                      uint8_t *reg_data,
                                      uint32_t length,
                                      uint32_t *actual_len,
                                      struct bhy2_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status, tmp_buf;
    int8_t rslt;

    /* Enter synchronous mode */
    rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
    if (rslt == BHY2_OK)
    {
        old_status = tmp_buf;
        tmp_buf &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL));
        rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHY2_OK)
        {
            /* Execute the Software passthrough */
            rslt = bhy2_hif_exec_cmd(BHY2_CMD_SW_PASSTHROUGH, payload, payload_len, hif);
            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_wait_status_ready(hif);
                if (rslt == BHY2_OK)
                {
                    /* Read back the response */
                    rslt = bhy2_hif_get_status_fifo(&code, reg_data, length, actual_len, hif);
                    if (rslt == BHY2_OK)
                    {
                        /* Set back to previous mode */
                        tmp_buf = old_status;
                        rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                        if (rslt == BHY2_OK)
                        {
                            if (code != BHY2_STATUS_SW_PASS_THRU_RES)
                            {
                                rslt = BHY2_E_TIMEOUT;
                            }
                        }
                    }
                }
            }
        }
    }

    return rslt;
}

int8_t bhy2_hif_get_post_mortem(uint16_t *code,
                                uint32_t *actual_len,
                                uint8_t *post_mortem,
                                uint32_t buffer_len,
                                struct bhy2_hif_dev *hif)
{
    uint8_t old_status, tmp_buf;
    int8_t rslt;

    if (code != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHY2_OK)
        {
            old_status = tmp_buf;
            tmp_buf &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_exec_cmd(BHY2_CMD_REQ_POST_MORTEM_DATA, NULL, 0, hif);
                if (rslt == BHY2_OK)
                {
                    rslt = bhy2_hif_wait_status_ready(hif);
                    if (rslt == BHY2_OK)
                    {
                        rslt = bhy2_hif_get_status_fifo(code, post_mortem, buffer_len, actual_len, hif);
                        if (rslt == BHY2_OK)
                        {
                            tmp_buf = old_status;
                            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                            if (rslt == BHY2_OK)
                            {
                                if (*code != BHY2_STATUS_CRASH_DUMP)
                                {
                                    rslt = BHY2_E_TIMEOUT;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_do_self_test(uint8_t phys_sensor_id,
                             struct bhy2_self_test_resp *self_test_resp,
                             struct bhy2_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status, tmp_buf, req[4], ret_data[16];
    uint32_t ret_len = 0;
    int8_t rslt;

    if (self_test_resp != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHY2_OK)
        {
            old_status = tmp_buf;
            tmp_buf &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
            if (rslt == BHY2_OK)
            {
                req[0] = phys_sensor_id;
                req[1] = req[2] = req[3] = 0;
                rslt = bhy2_hif_exec_cmd(BHY2_CMD_REQ_SELF_TEST, req, 4, hif);
                if (rslt == BHY2_OK)
                {
                    /* wait status ready */
                    rslt = bhy2_hif_wait_status_ready(hif);
                    if (rslt == BHY2_OK)
                    {
                        rslt = bhy2_hif_get_status_fifo(&code, ret_data, 16, &ret_len, hif);
                        if (rslt == BHY2_OK)
                        {
                            tmp_buf = old_status;
                            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                            if (rslt == BHY2_OK)
                            {
                                if (code != BHY2_STATUS_SELF_TEST_RES)
                                {
                                    rslt = BHY2_E_TIMEOUT;
                                }
                                else
                                {
                                    if (ret_data[0] != phys_sensor_id)
                                    {
                                        rslt = BHY2_E_INVALID_PARAM;
                                    }
                                    else
                                    {
                                        self_test_resp->test_status = ret_data[1];
                                        self_test_resp->x_offset = BHY2_LE2S16(&ret_data[2]);
                                        self_test_resp->y_offset = BHY2_LE2S16(&ret_data[4]);
                                        self_test_resp->z_offset = BHY2_LE2S16(&ret_data[6]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_do_foc(uint8_t phys_sensor_id, struct bhy2_foc_resp *foc_resp, struct bhy2_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status, tmp_buf, req[4], ret_data[16];
    uint32_t ret_len = 0;
    int8_t rslt;

    if (foc_resp != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHY2_OK)
        {
            old_status = tmp_buf;
            tmp_buf &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
            if (rslt == BHY2_OK)
            {
                req[0] = phys_sensor_id;
                req[1] = req[2] = req[3] = 0;
                rslt = bhy2_hif_exec_cmd(BHY2_CMD_REQ_FOC, req, 4, hif);
                if (rslt == BHY2_OK)
                {
                    rslt = bhy2_hif_wait_status_ready(hif);
                    if (rslt == BHY2_OK)
                    {
                        rslt = bhy2_hif_get_status_fifo(&code, ret_data, 16, &ret_len, hif);
                        if (rslt == BHY2_OK)
                        {
                            tmp_buf = old_status;
                            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                            if (rslt == BHY2_OK)
                            {
                                if (code != BHY2_STATUS_FOC_RES)
                                {
                                    rslt = BHY2_E_TIMEOUT;
                                }
                                else
                                {
                                    if (ret_data[0] != phys_sensor_id)
                                    {
                                        rslt = BHY2_E_INVALID_PARAM;
                                    }
                                    else
                                    {
                                        foc_resp->foc_status = ret_data[1];
                                        foc_resp->x_offset = BHY2_LE2S16(&ret_data[2]);
                                        foc_resp->y_offset = BHY2_LE2S16(&ret_data[4]);
                                        foc_resp->z_offset = BHY2_LE2S16(&ret_data[6]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_sensor_ctrl(uint8_t sensor_id,
                                uint8_t ctrl_code,
                                uint8_t *payload,
                                uint32_t payload_len,
                                uint32_t *actual_len,
                                struct bhy2_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status;
    uint8_t tmp_buf[4] = {0};
    uint8_t buf = 0;
    uint32_t length;
    uint16_t param_num;
    int8_t rslt;

    if ((payload != NULL) && (actual_len != NULL))
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
        if (rslt == BHY2_OK)
        {
            old_status = buf;
            buf &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
            if (rslt == BHY2_OK)
            {
                /* Set the parameter to request data */
                param_num = (uint16_t)(BHY2_PARAM_SET_SENSOR_CTRL | sensor_id);
                tmp_buf[0] = ctrl_code | BHY2_PARAM_SENSOR_CTRL_READ;
                length = 4;
                rslt = bhy2_hif_set_parameter(param_num, tmp_buf, length, hif);
                if (rslt == BHY2_OK)
                {
                    /* Get parameter to read data */
                    param_num = (uint16_t)(BHY2_PARAM_GET_SENSOR_CTRL | sensor_id);
                    length = 0;
                    rslt = bhy2_hif_set_parameter(param_num, NULL, length, hif);
                    if (rslt == BHY2_OK)
                    {
                        rslt = bhy2_hif_wait_status_ready(hif);
                        if (rslt == BHY2_OK)
                        {
                            rslt = bhy2_hif_get_status_fifo(&code, payload, payload_len, actual_len, hif);
                            if (rslt == BHY2_OK)
                            {
                                buf = old_status;
                                rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
                                if (rslt == BHY2_OK)
                                {
                                    if (code != (BHY2_PARAM_SET_SENSOR_CTRL | sensor_id))
                                    {
                                        rslt = BHY2_E_INVALID_PARAM;
                                    }
                                    else if (payload[0] != ctrl_code)
                                    {
                                        rslt = BHY2_E_INVALID_PARAM;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_set_sensor_ctrl(uint8_t sensor_id,
                                uint8_t ctrl_code,
                                const uint8_t *payload,
                                uint32_t payload_len,
                                struct bhy2_hif_dev *hif)
{
    uint16_t param_num;
    uint8_t command;

    /* Set parameter to send data */
    param_num = (uint16_t)(BHY2_PARAM_SET_SENSOR_CTRL | sensor_id);
    command = (uint8_t)(ctrl_code & (~BHY2_PARAM_SENSOR_CTRL_READ));

    return bhy2_hif_exec_cmd_generic(param_num, payload, payload_len, &command, 1, 0, hif);
}

int8_t bhy2_hif_get_bsx_state(uint16_t param_id,
                              uint8_t *bsx_state,
                              uint32_t state_len,
                              uint32_t *actual_len,
                              struct bhy2_hif_dev *hif)
{
    uint8_t complete_flag = 0;
    uint16_t tmp_state_len = 0;
    uint8_t block_len = 0;
    uint32_t read_len;
    int8_t rslt;
    uint8_t tmp_bsx_state_buf[BHY2_BSX_STATE_STRUCT_LEN];
    uint16_t state_index = 0;

    while (complete_flag == 0)
    {
        rslt = bhy2_hif_get_parameter(param_id, tmp_bsx_state_buf, BHY2_BSX_STATE_STRUCT_LEN, &read_len, hif);
        if (rslt != BHY2_OK)
        {
            break;
        }

        if (read_len > BHY2_BSX_STATE_STRUCT_LEN)
        {
            rslt = BHY2_E_INVALID_PARAM - 50;
            break;
        }

        complete_flag = tmp_bsx_state_buf[0] & BHY2_BSX_STATE_TRANSFER_COMPLETE;
        block_len = tmp_bsx_state_buf[1];
        tmp_state_len = BHY2_LE2U16(&tmp_bsx_state_buf[2]);
        if (tmp_state_len == 0)
        {
            rslt = BHY2_E_INVALID_PARAM - 70;
            break;
        }

        *actual_len = tmp_state_len; /* Report actual length, in case of invalid buffer */

        if (state_len < tmp_state_len)
        {
            rslt = BHY2_E_BUFFER;
            break;
        }

        memcpy(&bsx_state[state_index], &tmp_bsx_state_buf[4], tmp_bsx_state_buf[1]);
        state_index += block_len;
    }

    return rslt;
}

int8_t bhy2_hif_set_bsx_state(uint16_t param_id, const uint8_t *bsx_state, uint32_t state_len, struct bhy2_hif_dev *hif)
{
    uint32_t pos;
    uint16_t num = 0;
    int8_t rslt = BHY2_OK;
    uint8_t tmp_bsx_state_buf[BHY2_BSX_STATE_STRUCT_LEN];
    uint32_t state_buf_size;

    for (pos = 0; pos < state_len; pos += BHY2_BSX_STATE_BLOCK_LEN, num++)
    {
        tmp_bsx_state_buf[0] = num & 0x7F;
        state_buf_size = state_len - pos;
        if (state_buf_size <= BHY2_BSX_STATE_BLOCK_LEN)
        {
            tmp_bsx_state_buf[0] |= 0x80;
            tmp_bsx_state_buf[1] = (uint8_t)state_buf_size;
        }
        else
        {
            tmp_bsx_state_buf[1] = BHY2_BSX_STATE_BLOCK_LEN;
        }

        tmp_bsx_state_buf[2] = state_len & 0xFF;
        tmp_bsx_state_buf[3] = (state_len >> 8) & 0xFF;
        memcpy(&tmp_bsx_state_buf[4], &bsx_state[pos], tmp_bsx_state_buf[1]);
        rslt = bhy2_hif_set_parameter(param_id, tmp_bsx_state_buf, BHY2_BSX_STATE_STRUCT_LEN, hif);
        if (rslt != BHY2_OK)
        {
            break;
        }
    }

    return rslt;
}

int8_t bhy2_hif_erase_flash(uint32_t start_addr, uint32_t end_addr, struct bhy2_hif_dev *hif)
{
    uint8_t buf[8];
    uint8_t int_status = 0;
    uint16_t retry;

    /* Variable to get the response code */
    uint16_t code = 0;
    int8_t rslt = BHY2_OK;
    uint32_t ret_len;

    /* Check for invalid end address */
    if ((end_addr < start_addr) && (start_addr != BHY2_FLASH_BULK_ERASE))
    {
        rslt = BHY2_E_INVALID_PARAM;
    }
    else
    {
        /* Set start address */
        buf[0] = (uint8_t)(start_addr & 0xFF);
        buf[1] = (uint8_t)((start_addr >> 8) & 0xFF);
        buf[2] = (uint8_t)((start_addr >> 16) & 0xFF);
        buf[3] = (uint8_t)((start_addr >> 24) & 0xFF);

        /* Set the end address */
        buf[4] = (uint8_t)(end_addr & 0xFF);
        buf[5] = (uint8_t)((end_addr >> 8) & 0xFF);
        buf[6] = (uint8_t)((end_addr >> 16) & 0xFF);
        buf[7] = (uint8_t)((end_addr >> 24) & 0xFF);

        /* Erase flash */
        rslt = bhy2_hif_exec_cmd(BHY2_CMD_ERASE_FLASH, buf, 8, hif);
        if (rslt == BHY2_OK)
        {
            /* Check status and wait for at least 0.5 seconds for the erase to complete
             * and time out provided for 10 Mins (1200 * 0.5 Seconds = 10 Min) */
            for (retry = 0; retry < BHY2_QUERY_FLASH_MAX_RETRY; retry++)
            {
                rslt = bhy2_hif_get_interrupt_status(&int_status, hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }

                if (int_status & BHY2_IST_MASK_STATUS)
                {
                    break;
                }

                rslt = bhy2_hif_delay_us(500000, hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }
            }

            if (rslt == BHY2_OK)
            {
                /* Read status */
                rslt = bhy2_hif_get_status_fifo(&code, NULL, 0, &ret_len, hif);
                if (rslt == BHY2_OK)
                {
                    /* Status code */
                    if (code != BHY2_STATUS_FLASH_ERASE_COMPLETE)
                    {
                        rslt = BHY2_E_TIMEOUT;
                    }
                }
            }
        }
    }

    return rslt;
}

int8_t bhy2_hif_upload_to_flash(const uint8_t *firmware,
                                uint32_t length,
                                uint8_t *work_buffer,
                                uint32_t work_buf_len,
                                uint32_t *exp_size,
                                struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint16_t magic;
    uint32_t sector_addr = BHY2_FLASH_SECTOR_START_ADDR;
    uint8_t addr_buf[4];
    uint32_t remain, trans_len, pos;
    uint16_t code;

    if ((hif != NULL) && (firmware != NULL) && (work_buffer != NULL))
    {
        magic = BHY2_LE2U16(firmware);
        if (magic != BHY2_FW_MAGIC)
        {
            rslt = BHY2_E_MAGIC;
        }
        else
        {
            pos = 0;
            remain = length;
            if (hif->read_write_len % 4)
            {

                hif->read_write_len = BHY2_ROUND_WORD_LOWER(hif->read_write_len);
            }

            while (remain > 0)
            {
                if (remain > hif->read_write_len)
                {
                    trans_len = hif->read_write_len;
                }
                else
                {
                    trans_len = remain;
                }

                /* Set the sector address */
                addr_buf[0] = (uint8_t)(sector_addr & 0xFF);
                addr_buf[1] = (uint8_t)((sector_addr >> 8) & 0xFF);
                addr_buf[2] = (uint8_t)((sector_addr >> 16) & 0xFF);
                addr_buf[3] = (uint8_t)((sector_addr >> 24) & 0xFF);

                /* Transfer firmware to the flash */
                rslt = bhy2_hif_exec_cmd_generic(BHY2_CMD_WRITE_FLASH, &firmware[pos], trans_len, addr_buf, 4, 0, hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }

                rslt = bhy2_hif_wait_status_ready(hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }

                rslt = bhy2_hif_get_status_fifo(&code, work_buffer, work_buf_len, exp_size, hif);
                if (rslt != BHY2_OK)
                {
                    break;
                }

                /* Status code */
                if (code != BHY2_STATUS_FLASH_WRITE_COMPLETE)
                {
                    rslt = BHY2_E_TIMEOUT;
                    break;
                }

                pos += trans_len;
                remain -= trans_len;
                sector_addr += hif->read_write_len;
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_upload_to_flash_partly(const uint8_t *firmware,
                                       uint32_t cur_pos,
                                       uint32_t packet_len,
                                       struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint16_t magic;
    uint32_t sector_addr = BHY2_FLASH_SECTOR_START_ADDR + cur_pos;
    uint8_t addr_buf[4];
    uint16_t code;
    uint8_t work_buffer[32];
    uint32_t exp_size, work_buf_len = 32;

    if ((hif != NULL) && (firmware != NULL))
    {
        if (cur_pos == 0)
        {
            magic = BHY2_LE2U16(firmware);
            if (magic != BHY2_FW_MAGIC)
            {
                rslt = BHY2_E_MAGIC;
            }
        }

        if (rslt == BHY2_OK)
        {
            /* Set the sector address */
            addr_buf[0] = (uint8_t)(sector_addr & 0xFF);
            addr_buf[1] = (uint8_t)((sector_addr >> 8) & 0xFF);
            addr_buf[2] = (uint8_t)((sector_addr >> 16) & 0xFF);
            addr_buf[3] = (uint8_t)((sector_addr >> 24) & 0xFF);

            /* Transfer firmware to the flash */
            rslt = bhy2_hif_exec_cmd_generic(BHY2_CMD_WRITE_FLASH, firmware, packet_len, addr_buf, 4, 0, hif);
            if (rslt == BHY2_OK)
            {
                rslt = bhy2_hif_wait_status_ready(hif);
                if (rslt == BHY2_OK)
                {
                    rslt = bhy2_hif_get_status_fifo(&code, work_buffer, work_buf_len, &exp_size, hif);
                    if (rslt == BHY2_OK)
                    {
                        if (code != BHY2_STATUS_FLASH_WRITE_COMPLETE)
                        {
                            rslt = BHY2_E_TIMEOUT;
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_boot_from_flash(struct bhy2_hif_dev *hif)
{
    int8_t rslt;

    rslt = bhy2_hif_exec_cmd(BHY2_CMD_BOOT_FLASH, NULL, 0, hif);
    if (rslt == BHY2_OK)
    {
        rslt = bhy2_hif_check_boot_status_flash(hif);
    }

    return rslt;
}

int8_t bhy2_hif_request_hw_timestamp(struct bhy2_hif_dev *hif)
{
    uint8_t time_ev_req = 1;

    return bhy2_hif_set_regs(BHY2_REG_TIME_EV_REQ, &time_ev_req, 1, hif);
}

int8_t bhy2_hif_get_hw_timestamp(uint64_t *ts_ticks, struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHY2_OK;
    uint8_t ts_regs[5];

    if (ts_ticks != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTR_TIME_0, ts_regs, 5, hif);
        if (rslt == BHY2_OK)
        {
            *ts_ticks = BHY2_LE2U40(ts_regs);
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_req_and_get_hw_timestamp(uint64_t *ts_ticks, struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint8_t retry;
    uint8_t tmp_buf[5];
    uint64_t ts_old;

    if (ts_ticks != NULL)
    {
        rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTR_TIME_0, tmp_buf, 5, hif);
        if (rslt == BHY2_OK)
        {
            ts_old = BHY2_LE2U40(tmp_buf);
            tmp_buf[0] = 1;
            rslt = bhy2_hif_set_regs(BHY2_REG_TIME_EV_REQ, tmp_buf, 1, hif);
            if (rslt == BHY2_OK)
            {
                for (retry = 0; retry < 5; retry++)
                {
                    rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTR_TIME_0, tmp_buf, 5, hif);
                    if (rslt == BHY2_OK)
                    {
                        *ts_ticks = BHY2_LE2U40(tmp_buf);
                        if (*ts_ticks != ts_old)
                        {
                            break;
                        }

                        rslt = bhy2_hif_delay_us(10, hif);
                        if (rslt != BHY2_OK)
                        {
                            break;
                        }
                    }
                }

                if ((retry >= 5) && (*ts_ticks == ts_old))
                {
                    rslt = BHY2_E_TIMEOUT;
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_sensor_info(uint8_t sensor_id, struct bhy2_sensor_info *info, struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint32_t length;
    uint8_t bytes[28];

    if (info != NULL)
    {
        /* Checking for a valid sensor ID */
        if (sensor_id > BHY2_SPECIAL_SENSOR_ID_OFFSET)
        {
            rslt = BHY2_E_INVALID_PARAM;
        }
        else
        {
            rslt = bhy2_hif_get_parameter((uint16_t)(BHY2_PARAM_SENSOR_INFO_0 + sensor_id), bytes, 28, &length, hif);
            if (rslt == BHY2_OK)
            {
                if (length != 28)
                {
                    rslt = BHY2_E_INVALID_PARAM;
                }
                else
                {
                    info->sensor_type = bytes[0];
                    info->driver_id = bytes[1];
                    info->driver_version = bytes[2];
                    info->power = bytes[3];
                    info->max_range.bytes[0] = bytes[4];
                    info->max_range.bytes[1] = bytes[5];
                    info->resolution.bytes[0] = bytes[6];
                    info->resolution.bytes[1] = bytes[7];
                    info->max_rate.bytes[0] = bytes[8];
                    info->max_rate.bytes[1] = bytes[9];
                    info->max_rate.bytes[2] = bytes[10];
                    info->max_rate.bytes[3] = bytes[11];
                    info->fifo_reserved.bytes[0] = bytes[12];
                    info->fifo_reserved.bytes[1] = bytes[13];
                    info->fifo_reserved.bytes[2] = bytes[14];
                    info->fifo_reserved.bytes[3] = bytes[15];
                    info->fifo_max.bytes[0] = bytes[16];
                    info->fifo_max.bytes[1] = bytes[17];
                    info->fifo_max.bytes[2] = bytes[18];
                    info->fifo_max.bytes[3] = bytes[19];
                    info->event_size = bytes[20];
                    info->min_rate.bytes[0] = bytes[21];
                    info->min_rate.bytes[1] = bytes[22];
                    info->min_rate.bytes[2] = bytes[23];
                    info->min_rate.bytes[3] = bytes[24];
                    info->max_range.u16_val = BHY2_LE2U16(info->max_range.bytes);
                    info->resolution.u16_val = BHY2_LE2U16(info->resolution.bytes);
                    info->max_rate.u32_val = BHY2_LE2U32(info->max_rate.bytes);
                    info->fifo_reserved.u32_val = BHY2_LE2U32(info->fifo_reserved.bytes);
                    info->fifo_max.u32_val = BHY2_LE2U32(info->fifo_max.bytes);
                    info->min_rate.u32_val = BHY2_LE2U32(info->min_rate.bytes);
                }
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_get_phys_sensor_info(uint8_t sensor_id, struct bhy2_phys_sensor_info *info, struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint32_t length;
    uint8_t bytes[20];

    if (info != NULL)
    {
        rslt = bhy2_hif_get_parameter((uint16_t)(BHY2_PARAM_PHYSICAL_SENSOR_BASE + sensor_id), bytes, 20, &length, hif);
        if (rslt == BHY2_OK)
        {
            if (length != 20)
            {
                rslt = BHY2_E_INVALID_PARAM;
            }
            else
            {
                info->sensor_type = bytes[0];
                info->driver_id = bytes[1];
                info->driver_version = bytes[2];
                info->power_current = bytes[3];
                info->curr_range.bytes[0] = bytes[4];
                info->curr_range.bytes[1] = bytes[5];
                info->flags = bytes[6];
                info->slave_address = bytes[7];
                info->gpio_assignment = bytes[8];
                info->curr_rate.bytes[0] = bytes[9];
                info->curr_rate.bytes[1] = bytes[10];
                info->curr_rate.bytes[2] = bytes[11];
                info->curr_rate.bytes[3] = bytes[12];
                info->num_axis = bytes[13];
                info->orientation_matrix[0] = bytes[14];
                info->orientation_matrix[1] = bytes[15];
                info->orientation_matrix[2] = bytes[16];
                info->orientation_matrix[3] = bytes[17];
                info->orientation_matrix[4] = bytes[18];

                info->curr_range.u16_val = BHY2_LE2U16(info->curr_range.bytes);
                info->curr_rate.u32_val = BHY2_LE2U32(info->curr_rate.bytes);
            }
        }
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhy2_hif_set_orientation_matrix(uint8_t sensor_id,
                                       struct bhy2_orient_matrix orient_matrix,
                                       struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint8_t tmp_matrix_buf[8]; /* Includes 2 reserved bytes */

    memset(tmp_matrix_buf, 0, sizeof(tmp_matrix_buf));

    tmp_matrix_buf[0] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[0]);
    tmp_matrix_buf[1] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[2]);
    tmp_matrix_buf[2] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[4]);
    tmp_matrix_buf[3] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[6]);
    tmp_matrix_buf[4] = orient_matrix.c[8] & 0x0F;

    rslt = bhy2_hif_set_parameter((uint16_t)(BHY2_PARAM_PHYSICAL_SENSOR_BASE + sensor_id), tmp_matrix_buf, 8, hif);

    return rslt;
}

/*lint -e{715} suppressed parameters not referenced info as all the calling functions are parsing the valid values to
 * all parameters*/
int8_t bhy2_hif_set_inject_data_mode(const uint8_t *payload,
                                     uint8_t payload_len,
                                     const uint8_t *work_buf,
                                     uint32_t work_buf_len,
                                     const uint32_t *actual_len,
                                     struct bhy2_hif_dev *hif)
{
    uint8_t tmp_buf;
    int8_t rslt;

    rslt = bhy2_hif_get_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
    if (rslt == BHY2_OK)
    {
        tmp_buf &= (uint8_t)(~(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL));
        rslt = bhy2_hif_set_regs(BHY2_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHY2_OK)
        {
            /* Set injection mode */
            rslt = bhy2_hif_set_regs(BHY2_REG_CHAN_CMD, payload, payload_len, hif);
            if (rslt != BHY2_OK)
            {
                rslt = BHY2_E_NULL_PTR;
            }
        }
    }

    return rslt;
}

int8_t bhy2_hif_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhy2_hif_dev *hif)
{
    return bhy2_hif_exec_cmd(BHY2_CMD_INJECT_DATA, payload, payload_len, hif);
}
