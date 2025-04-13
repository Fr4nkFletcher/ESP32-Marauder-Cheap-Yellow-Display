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
* @file       bhy2_hif.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHY2_HIF_H__
#define __BHY2_HIF_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhy2_defs.h"

/**
 * @brief Function to link the callback interfaces
 * @param[in] intf              : Physical communication interface
 * @param[in] read              : Read function pointer
 * @param[in] write             : Write function pointer
 * @param[in] delay_us          : Microsecond delay function pointer
 * @param[in] read_write_len    : Maximum read/write lengths supported
 * @param[in] intf_ptr          : Reference to the interface. Can be NULL
 * @param[in] hif               : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_init(enum bhy2_intf intf,
                     bhy2_read_fptr_t read,
                     bhy2_write_fptr_t write,
                     bhy2_delay_us_fptr_t delay_us,
                     uint32_t read_write_len,
                     void *intf_ptr,
                     struct bhy2_hif_dev *hif);

/**
 * @brief Function to get data from registers
 * @param[in] reg_addr  : Register address to be read from
 * @param[out] reg_data : Reference to the data buffer
 * @param[in] length    : Length of the data buffer
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bhy2_hif_dev *hif);

/**
 * @brief Function to set data to registers
 * @param[in] reg_addr  : Register address to write to
 * @param[in] reg_data  : Reference to the data buffer
 * @param[in] length    : Length of the data buffer
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bhy2_hif_dev *hif);

/**
 * @brief Function to delay in microseconds
 * @param[in] period_us : Period to delay in microseconds
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_delay_us(uint32_t period_us, const struct bhy2_hif_dev *hif);

/**
 * Function to execute a command
 * @param[in] cmd       : Command code
 * @param[in] payload   : Reference to the data buffer containing the command's payload
 * @param[in] length    : Length of the data buffer
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_exec_cmd(uint16_t cmd, const uint8_t *payload, uint32_t length, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get a parameter
 * @param[in] param         : Parameter ID
 * @param[out] payload      : Reference to the data buffer to store the parameter's payload
 * @param[in] payload_len   : Length of the data buffer
 * @param[out] actual_len   : Actual length of the payload
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_parameter(uint16_t param,
                              uint8_t *payload,
                              uint32_t payload_len,
                              uint32_t *actual_len,
                              struct bhy2_hif_dev *hif);

/**
 * @brief Function to set a parameter
 * @param[in] param     : Parameter ID
 * @param[in] payload   : Reference to the data buffer storing the parameter's payload
 * @param[in] length    : Length of the data buffer
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_parameter(uint16_t param, const uint8_t *payload, uint32_t length, struct bhy2_hif_dev *hif);

/**
 * @brief Function to erase the flash
 * @param[in] start_addr    : Start address
 * @param[in] end_addr      : End address
 * @param[in] hif           : HIF device address
 * @return API error codes
 */
int8_t bhy2_hif_erase_flash(uint32_t start_addr, uint32_t end_addr, struct bhy2_hif_dev *hif);

/**
 * @brief Function to upload data to flash
 * @param[in] firmware          : Reference to the data buffer storing the firmware
 * @param[in] length            : Length of the data buffer
 * @param[in/out] work_buffer   : Reference to the work buffer
 * @param[in] work_buf_len      : Length of the work buffer
 * @param[out] exp_size         : Expected size of the work buffer if insufficient size
 * @param[in] hif               : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_upload_to_flash(const uint8_t *firmware,
                                uint32_t length,
                                uint8_t *work_buffer,
                                uint32_t work_buf_len,
                                uint32_t *exp_size,
                                struct bhy2_hif_dev *hif);

/**
 * @brief Function to upload data to flash partly
 * @param[in] firmware      : Reference cur_pos of the data buffer storing the firmware
 * @param[in] cur_pos       : Current position of the firmware
 * @param[in] packet_len    : Length of the firmware packet to be transferred
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_upload_to_flash_partly(const uint8_t *firmware,
                                       uint32_t cur_pos,
                                       uint32_t packet_len,
                                       struct bhy2_hif_dev *hif);

/**
 * @brief Function to boot from flash
 * @param[in] hif   : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_boot_from_flash(struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the product ID
 * @param[out] product_id   : Reference to the data buffer to store the product ID
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_product_id(uint8_t *product_id, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the chip ID
 * @param[out] chip_id  : Reference to the data buffer to store the chip ID
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_chip_id(uint8_t *chip_id, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the ROM version
 * @param[out] rom_version  : Reference to the data buffer to store the ROM version
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_rom_version(uint16_t *rom_version, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the kernel version
 * @param[out] kernel_version   : Reference to the data buffer to store the kernel version
 * @param[in] hif               : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_kernel_version(uint16_t *kernel_version, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the user version
 * @param[out] user_version : Reference to the data buffer to store the user version
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_user_version(uint16_t *user_version, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the boot status
 * @param[out] boot_status  : Reference to the data buffer to store the boot status
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_boot_status(uint8_t *boot_status, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the host status
 * @param[out] host_status  : Reference to the data buffer to store the host status
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_host_status(uint8_t *host_status, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the feature status
 * @param[out] feat_status  : Reference to the data buffer to store the feature status
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_feature_status(uint8_t *feat_status, struct bhy2_hif_dev *hif);

/**
 * @brief Function get the interrupt status
 * @param[out] int_status   : Reference to the data buffer to store the interrupt status
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_interrupt_status(uint8_t *int_status, struct bhy2_hif_dev *hif);

/**
 * @brief Function get the firmware error code
 * @param[out] fw_error : Reference to the data buffer to store the firmware error code
 * @param[in] hif       : HIF device reference
 * @return API error code
 */
int8_t bhy2_hif_get_fw_error(uint8_t *fw_error, struct bhy2_hif_dev *hif);

/**
 * @brief Function to trigger a soft reset
 * @param[in] hif : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_reset(struct bhy2_hif_dev *hif);

/**
 * @brief Function to upload a firmware to RAM
 * @param[in] firmware  : Reference to the data buffer storing the firmware
 * @param[in] length    : Length of the firmware
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_upload_firmware_to_ram(const uint8_t *firmware, uint32_t length, struct bhy2_hif_dev *hif);

/**
 * @brief Function to upload a part of the firmware to RAM
 * @param[in] firmware      : Reference to the data buffer storing the current part of firmware
 * @param[in] total_size    : Total size of the firmware
 * @param[in] cur_pos       : Current position of the firmware
 * @param[in] packet_len    : Size of firmware part to be uploaded
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_upload_firmware_to_ram_partly(const uint8_t *firmware,
                                              uint32_t total_size,
                                              uint32_t cur_pos,
                                              uint32_t packet_len,
                                              struct bhy2_hif_dev *hif);

/**
 * @brief Function to boot from RAM
 * @param[in] hif   : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_boot_program_ram(struct bhy2_hif_dev *hif);

/**
 * @brief Function to get data from the Wake-up FIFO
 * @param[out] fifo         : Reference to the data buffer to store data from the FIFO
 * @param[in] fifo_len      : Length of the data buffer
 * @param[out] bytes_read   : Number of bytes read into the data buffer
 * @param[out] bytes_remain : Bytes remaining in the sensor FIFO
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_wakeup_fifo(uint8_t *fifo,
                                uint32_t fifo_len,
                                uint32_t *bytes_read,
                                uint32_t *bytes_remain,
                                struct bhy2_hif_dev *hif);

/**
 * @brief Function to get data from the Non-wake-up FIFO
 * @param[out] fifo         : Reference to the data buffer to store data from the FIFO
 * @param[in] fifo_len      : Length of the data buffer
 * @param[out] bytes_read   : Number of bytes read into the data buffer
 * @param[out] bytes_remain : Bytes remaining in the sensor FIFO
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_nonwakeup_fifo(uint8_t *fifo,
                                   uint32_t fifo_len,
                                   uint32_t *bytes_read,
                                   uint32_t *bytes_remain,
                                   struct bhy2_hif_dev *hif);

/**
 * @brief Function to get synchronous data from the Status FIFO
 * @param[out] status_code  : Status code received
 * @param[out] fifo         : Reference to the data buffer to store the payload of the status code
 * @param[in] fifo_len      : Length of the data buffer
 * @param[out] bytes_remain : Number of bytes in the payload
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_status_fifo(uint16_t *status_code,
                                uint8_t *fifo,
                                uint32_t fifo_len,
                                uint32_t *bytes_remain,
                                struct bhy2_hif_dev *hif);

/**
 * @brief Function to get asynchronous data from the Status FIFO
 * @param[out] fifo         : Reference to the data buffer to store data from the FIFO
 * @param[in] fifo_len      : Length of the data buffer
 * @param[out] bytes_read   : Number of bytes read into the data buffer
 * @param[out] bytes_remain : Bytes remaining in the sensor FIFO
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_status_fifo_async(uint8_t *fifo,
                                      uint32_t fifo_len,
                                      uint32_t *bytes_read,
                                      uint32_t *bytes_remain,
                                      struct bhy2_hif_dev *hif);

/**
 * @brief Function to configure a virtual sensor
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] sample_rate   : Sample rate in Hz
 * @param[in] latency       : Report latency in milliseconds
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_exec_sensor_conf_cmd(uint8_t sensor_id,
                                     bhy2_float sample_rate,
                                     uint32_t latency,
                                     struct bhy2_hif_dev *hif);

/**
 * @brief Function to flush data of a virtual sensor from the FIFOs
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_fifo_flush(uint8_t sensor_id, struct bhy2_hif_dev *hif);

/**
 * @brief Function to perform a self-test for a particular virtual sensor
 * @param[in] phys_sensor_id    : Physical sensor ID of the virtual sensor
 * @param[out] self_test_resp   : Reference to the data buffer to store the self-test response
 * @param[in] hif               : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_do_self_test(uint8_t phys_sensor_id,
                             struct bhy2_self_test_resp *self_test_resp,
                             struct bhy2_hif_dev *hif);

/**
 * @brief Function to perform a fast offset compensation for a particular virtual sensor
 * @param[in] phys_sensor_id : Physical Sensor ID of the virtual sensor
 * @param[out] foc_resp      : Reference to the data buffer to store the FOC response
 * @param[in] hif            : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_do_foc(uint8_t phys_sensor_id, struct bhy2_foc_resp *foc_resp, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the sensor control configuration of a virtual sensor
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] ctrl_code     : Sensor control code
 * @param[out] payload      : Reference to the data buffer to store the sensor control
 * @param[in] payload_len   : Length of the data buffer
 * @param[out] actual_len   : Actual length of the sensor control configuration
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_sensor_ctrl(uint8_t sensor_id,
                                uint8_t ctrl_code,
                                uint8_t *payload,
                                uint32_t payload_len,
                                uint32_t *actual_len,
                                struct bhy2_hif_dev *hif);

/**
 * @brief Function to set the sensor control configuration of a virtual sensor
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] ctrl_code     : Sensor control code
 * @param[in] payload       : Reference to the data buffer storing the sensor control
 * @param[in] payload_len   : Length of the sensor control configuration
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_sensor_ctrl(uint8_t sensor_id,
                                uint8_t ctrl_code,
                                const uint8_t *payload,
                                uint32_t payload_len,
                                struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the BSX state of a BSX parameter
 * @param[in] param_id      : Parameter ID of the BSX parameter
 * @param[out] bsx_state    : Reference to teh data buffer to store the BSX state
 * @param[in] state_len     : Length of the buffer
 * @param[out] actual_len   : Actual length of the BSX state
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_bsx_state(uint16_t param_id,
                              uint8_t *bsx_state,
                              uint32_t state_len,
                              uint32_t *actual_len,
                              struct bhy2_hif_dev *hif);

/**
 * @brief Function to the BSX state of a BSX parameter
 * @param[in] param_id  : Parameter ID of the BSX parameter
 * @param[in] bsx_state : Reference to the data buffer storing the BSX state
 * @param[in] state_len : Length of the BSX state
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_bsx_state(uint16_t param_id, const uint8_t *bsx_state, uint32_t state_len,
                              struct bhy2_hif_dev *hif);

/**
 * @brief Function to request a timestamp event
 * @param[in] hif   : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_request_hw_timestamp(struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the hardware timestamp
 * @param[out] ts_ticks : Reference to the data buffer to store the timestamp in ticks
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_hw_timestamp(uint64_t *ts_ticks, struct bhy2_hif_dev *hif);

/**
 * @brief Function to request and get the hardware timestamp
 * @param[out] ts_ticks : Reference to the data buffer to store the timestamp in ticks
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_req_and_get_hw_timestamp(uint64_t *ts_ticks, struct bhy2_hif_dev *hif);

/**
 * @brief Function to communicate with connected sensors
 * @param[in] payload       : Command payload
 * @param[in] payload_len   : Payload length
 * @param[in/out] reg_data  : Reference to the data buffer storing the register data
 * @param[in] length        : Length of the data buffer
 * @param[out] actual_len   : Registers actually read
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_exec_soft_passthrough(const uint8_t *payload,
                                      uint32_t payload_len,
                                      uint8_t *reg_data,
                                      uint32_t length,
                                      uint32_t *actual_len,
                                      struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the post-mortem data dump
 * @param[out] code         : Post-mortem status code
 * @param[out] actual_len   : Actual length of the post-mortem data dump
 * @param[out] post_mortem  : Reference to the data buffer to store the data dump
 * @param[in] buffer_len    : Length of the data buffer
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_post_mortem(uint16_t *code,
                                uint32_t *actual_len,
                                uint8_t *post_mortem,
                                uint32_t buffer_len,
                                struct bhy2_hif_dev *hif);

/**
 * @brief Function get information of a virtual sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] info     : Reference to the data buffer to store the sensor info
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_sensor_info(uint8_t sensor_id, struct bhy2_sensor_info *info, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the physical sensor information of a virtual sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] info     : Reference to the data buffer to store the physical sensor info
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_get_phys_sensor_info(uint8_t sensor_id, struct bhy2_phys_sensor_info *info, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get the orientation matrix of a virtual sensor
 * @param[in] sensor_id         : Sensor ID of the virtual sensor
 * @param[out] orient_matrix    : Reference to the data buffer to store the orientation matrix
 * @param[in] hif               : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_orientation_matrix(uint8_t sensor_id,
                                       struct bhy2_orient_matrix orient_matrix,
                                       struct bhy2_hif_dev *hif);

/**
 * @brief Function to set the data injection mode
 * @param[in] payload       : Reference to the data buffer containing the host injection mode
 * @param[in] payload_len   : Length of the data buffer
 * @param[in/out] work_buf  : Reference to the work buffer
 * @param[in] work_buf_len  : Length of the work buffer
 * @param[out] actual_len   : Expected length of the work buffer
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_set_inject_data_mode(const uint8_t *payload,
                                     uint8_t payload_len,
                                     const uint8_t *work_buf,
                                     uint32_t work_buf_len,
                                     const uint32_t *actual_len,
                                     struct bhy2_hif_dev *hif);

/**
 * @brief Function to inject data
 * @param[in] payload       : Reference to the data buffer
 * @param[in] payload_len   : Length of the data buffer
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhy2_hif_dev *hif);

/**
 * @brief Function to wait till status is ready
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhy2_hif_wait_status_ready(struct bhy2_hif_dev *hif);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY2_HIF_H__ */
