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
* @file       bhy2.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHY2_H__
#define __BHY2_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhy2_defs.h"

/**
 * @brief Function to link the callback interfaces
 * @param[in] intf          : Physical communication interface
 * @param[in] read          : Read function pointer
 * @param[in] write         : Write function pointer
 * @param[in] delay_us      : Microsecond delay function pointer
 * @param[in] read_write_len : Maximum read/write lengths supported
 * @param[in] intf_ptr      : Reference to the interface. Can be NULL
 * @param[out] dev          : Device reference
 * @return API error codes
 */
int8_t bhy2_init(enum bhy2_intf intf,
                 bhy2_read_fptr_t read,
                 bhy2_write_fptr_t write,
                 bhy2_delay_us_fptr_t delay_us,
                 uint32_t read_write_len,
                 void *intf_ptr,
                 struct bhy2_dev *dev);

/**
 * @brief Function to get data from registers
 * @param[in] reg_addr  : Register address to be read from
 * @param[out] reg_data : Reference to the data buffer
 * @param[in] length    : Length of the data buffer
 * @param[in] dev       : Device reference
 * @return API Error codes
 */
int8_t bhy2_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, struct bhy2_dev *dev);

/**
 * @brief Function to set data to registers
 * @param[in] reg_addr  : Register address to write to
 * @param[in] reg_data  : Reference to the data buffer
 * @param[in] length    : Length of the data buffer
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t length, struct bhy2_dev *dev);

/**
 * @brief Function to get the product ID
 * @param[out] product_id   : Reference to data buffer to store the product ID
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_product_id(uint8_t *product_id, struct bhy2_dev *dev);

/**
 * @brief Function to get the chip ID
 * @param[out] chip_id  : Reference to data buffer to store the chip ID
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_get_chip_id(uint8_t *chip_id, struct bhy2_dev *dev);

/**
 * @brief Function to get the revision ID
 * @param[out] revision_id  : Reference to data buffer to store the revision ID
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_revision_id(uint8_t *revision_id, struct bhy2_dev *dev);

/**
 * @brief Function to get the ROM version
 * @param[out] rom_version  : Reference to the data buffer to store the ROM version
 * @param[in] dev           : Device reference
 * @return  API error codes
 */
int8_t bhy2_get_rom_version(uint16_t *rom_version, struct bhy2_dev *dev);

/**
 * @brief Function to get the kernel version
 * @param[out] kernel_version   : Reference to the data buffer to store the kernel version
 * @param[in] dev               : Device reference
 * @return API error codes
 */
int8_t bhy2_get_kernel_version(uint16_t *kernel_version, struct bhy2_dev *dev);

/**
 * @brief Function to get the user version
 * @param[out] user_version : Reference to the data buffer to store the user version
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_user_version(uint16_t *user_version, struct bhy2_dev *dev);

/**
 * @brief Function to get the boot status
 * @param[out] boot_status  : Reference to the data buffer to store the boot status
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_boot_status(uint8_t *boot_status, struct bhy2_dev *dev);

/**
 * @brief Function to get the host status
 * @param[out] host_status  : Reference to the data buffer to store the host status
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_host_status(uint8_t *host_status, struct bhy2_dev *dev);

/**
 * @brief Function to get the feature status
 * @param[out] feat_status  : Reference to the data buffer to store the feature status
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_feature_status(uint8_t *feat_status, struct bhy2_dev *dev);

/**
 * @brief Function to get the virtual sensor list  and store in the device structure
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_virt_sensor_list(struct bhy2_dev *dev);

/**
 * @brief Function to get the virtual sensor configuration
 * @param[in] sensor_id         : Sensor ID of the virtual sensor
 * @param[out] virt_sensor_conf : Reference to the data buffer to store the virtual sensor configuration
 * @param[in] dev               : Device reference
 * @return API error codes
 */
int8_t bhy2_get_virt_sensor_cfg(uint8_t sensor_id, struct bhy2_virt_sensor_conf *virt_sensor_conf,
                                struct bhy2_dev *dev);

/**
 * @brief Function to set the sample rate and latency of the virtual sensor
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] sample_rate   : Sample rate in Hz
 * @param[in] latency       : Report latency in milliseconds
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_set_virt_sensor_cfg(uint8_t sensor_id, bhy2_float sample_rate, uint32_t latency, struct bhy2_dev *dev);

/**
 * @brief Function set the range of the virtual sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[in] range     : Range of the virtual sensor
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_virt_sensor_range(uint8_t sensor_id, uint16_t range, struct bhy2_dev *dev);

/**
 * @brief Function to get and process the FIFOs
 * @param[in] work_buffer   : Reference to the data buffer where the FIFO data is copied to before parsing
 * @param[in] buffer_size   : Size of the data buffer
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_and_process_fifo(uint8_t *work_buffer, uint32_t buffer_size, struct bhy2_dev *dev);

/**
 * @brief Function get the FIFO control register
 * @param[out] fifo_ctrl    : Reference to the data buffer to store the FIFO control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_fifo_ctrl(uint32_t *fifo_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to set the Wake up FIFO watermark
 * @param[in] watermark : Size of the watermark
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_fifo_wmark_wkup(uint32_t watermark, struct bhy2_dev *dev);

/**
 * @brief Function to get the Wake up FIFO watermark
 * @param[out] watermark    : Reference to the data buffer to store the FIFO watermark size
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_fifo_wmark_wkup(uint32_t *watermark, struct bhy2_dev *dev);

/**
 * @brief Function to set the Non wake up FIFO watermark
 * @param[in] watermark : Size of the watermark
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_fifo_wmark_nonwkup(uint32_t watermark, struct bhy2_dev *dev);

/**
 * @brief Function to get the Non wake up FIFO watermark
 * @param[out] watermark    : Reference to the data buffer to store the FIFO watermark size
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_fifo_wmark_nonwkup(uint32_t *watermark, struct bhy2_dev *dev);

/**
 * @brief Function to flush data of a virtual sensor from the FIFOs
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_flush_fifo(uint8_t sensor_id, struct bhy2_dev *dev);

/**
 * @brief Function to set the FIFO format
 * @param[in] param : FIFO format settings
 * @param[in] dev   : Device reference
 * @return API error codes
 */
int8_t bhy2_set_fifo_format_ctrl(uint8_t param, struct bhy2_dev *dev);

/**
 * @brief Function to upload firmware to RAM
 * @param[in] firmware  : Reference to the data buffer containing the firmware
 * @param[in] length    : Size of the firmware
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_upload_firmware_to_ram(const uint8_t *firmware, uint32_t length, struct bhy2_dev *dev);

/**
 * @brief Function to upload part of the firmware to RAM
 * @param[in] firmware      : Reference to the data buffer containing the current firmware section
 * @param[in] total_size    : Total size of the firmware
 * @param[in] cur_pos       : Current position of the part being loaded
 * @param[in] packet_len    : Size of the part being loaded
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_upload_firmware_to_ram_partly(const uint8_t *firmware,
                                          uint32_t total_size,
                                          uint32_t cur_pos,
                                          uint32_t packet_len,
                                          struct bhy2_dev *dev);

/**
 * @brief Function to boot firmware from RAM
 * @param[in] dev   : Device reference
 * @return API error codes
 */
int8_t bhy2_boot_from_ram(struct bhy2_dev *dev);

/**
 * @brief Function to erase the flash
 * @param[in] start_addr    : Start address
 * @param[in] end_addr      : End address
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_erase_flash(uint32_t start_address, uint32_t end_addr, struct bhy2_dev *dev);

/**
 * @brief Function to upload firmware to Flash
 * @param[in] firmware  : Reference to the data buffer containing the firmware
 * @param[in] length    : Size of the firwmare
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_upload_firmware_to_flash(const uint8_t *firmware, uint32_t length, struct bhy2_dev *dev);

/**
 * @brief Function to upload part of the firmware to Flash
 * @param[in] firmware      : Reference to the data buffer containing the current firmware section
 * @param[in] cur_pos       : Current position of the part being loaded
 * @param[in] packet_len    : Size of the part being loaded
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_upload_firmware_to_flash_partly(const uint8_t *firmware,
                                            uint32_t cur_pos,
                                            uint32_t packet_len,
                                            struct bhy2_dev *dev);

/**
 * @brief Function to boot from Flash
 * @param[in] dev   : Device reference
 * @return API error codes
 */
int8_t bhy2_boot_from_flash(struct bhy2_dev *dev);

/**
 * @brief Function to set the host interrupt control register
 * @param[in] hintr_ctrl    : Host interrupt control configuration
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_set_host_interrupt_ctrl(uint8_t hintr_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to get the host interrupt control register
 * @param[out] hintr_ctrl   : Reference to the data buffer to store the configuration
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_host_interrupt_ctrl(uint8_t *hintr_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to get the interrupt status register
 * @param[out] int_status   : Reference to the data buffer to store the interrupt status
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_interrupt_status(uint8_t *int_status, struct bhy2_dev *dev);

/**
 * @brief Function to set the host interface control register
 * @param[in] hintf_ctrl    : Host interface control configuration
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_set_host_intf_ctrl(uint8_t hintf_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to get the host interface control register
 * @param[out] hintf_ctrl   : Reference to the data buffer to store the configuration
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_host_intf_ctrl(uint8_t *hintf_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to trigger a timestamp event
 * @param[in] ts_ev_req : Pass a non-zero value to trigger a timestamp event
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_timestamp_event_req(uint8_t ts_ev_req, struct bhy2_dev *dev);

/**
 * @brief Function to get the timestamp of the sensor in nanoseconds
 * @param[out] timestamp_ns : Reference to the data buffer to store the timestamp in nanoseconds
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_hw_timestamp_ns(uint64_t *timestamp_ns, struct bhy2_dev *dev);

/**
 * @brief Function to get the host control register
 * @param[in] host_ctrl : Host control configuration
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_host_ctrl(uint8_t host_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to get the host control register
 * @param[out] host_ctrl    : Reference to the data buffer to store the host control configuration
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_host_ctrl(uint8_t *host_ctrl, struct bhy2_dev *dev);

/**
 * @brief Function to trigger a soft reset
 * @param[in] dev   : Device reference
 * @return API error codes
 */
int8_t bhy2_soft_reset(struct bhy2_dev *dev);

/**
 * @brief Function to perform a self test of a virtual sensor
 * @param[in] phys_sensor_id    : Physical sensor ID of the virtual sensor
 * @param[out] self_test_resp   : Reference to the data buffer to store the self test response from the virtual sensor
 * @param[in] dev               : Device reference
 * @return API error codes
 */
int8_t bhy2_perform_self_test(uint8_t phys_sensor_id, struct bhy2_self_test_resp *self_test_resp, struct bhy2_dev *dev);

/**
 * @brief Function to perform a fast offset compensation of a virtual sensor
 * @param[in] phys_sensor_id    : Physical Sensor ID of the virtual sensor
 * @param[out] self_test_resp   : Reference to the data buffer to store the FOC response from the virtual sensor
 * @param[in] dev               : Device reference
 * @return API error codes
 */
int8_t bhy2_perform_foc(uint8_t phys_sensor_id, struct bhy2_foc_resp *foc_resp, struct bhy2_dev *dev);

/**
 * @brief Function to set the orientation matrix of a physical sensor
 * @param[in] phys_sensor_id : Sensor ID of the virtual sensor
 * @param[in] orient_matrix  : Orientation matrix
 * @param[in] dev            : Device reference
 * @return API error codes
 */
int8_t bhy2_set_orientation_matrix(uint8_t phys_sensor_id, struct bhy2_orient_matrix orient_matrix,
                                   struct bhy2_dev *dev);

/**
 * @brief Function to get the physical sensor information
 * @param[in] phys_sensor_id : Sensor ID of the physical sensor
 * @param[out] phy_sen_info : Reference to the data buffer to the store the physical sensor information
 * @param[in] dev            : Device reference
 * @return API error codes
 */
int8_t bhy2_get_phys_sensor_info(uint8_t phys_sensor_id,
                                 struct bhy2_phys_sensor_info *phy_sen_info,
                                 struct bhy2_dev *dev);

/**
 * @brief Function to get the orientation matrix of a physical sensor
 * @param[in] phys_sensor_id : Sensor ID of the virtual sensor
 * @param[out] orient_matrix : Reference to the data buffer to the store the orientation matrix
 * @param[in] dev            : Device reference
 * @return API error codes
 */
int8_t bhy2_get_orientation_matrix(uint8_t phys_sensor_id,
                                   struct bhy2_orient_matrix *orient_matrix,
                                   struct bhy2_dev *dev);

/**
 * @brief Function to set the Soft Iron Correction matrix
 * @param[in] sic_matrix    : Reference to the data buffer storing the Soft Iron Correction matrix
 * @param[in] matrix_len    : Length of the matrix
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_set_sic_matrix(const uint8_t *sic_matrix, uint16_t matrix_len, struct bhy2_dev *dev);

/**
 * @brief Function to get the Soft Iron Correction matrix
 * @param[out] sic_matrix   : Reference to the data buffer to store the Soft Iron Correction matrix
 * @param[in] matrix_len    : Length of data buffer
 * @param[out] actual_len   : Actual length of the Soft Iron Correction matrix
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_sic_matrix(uint8_t *sic_matrix, uint16_t matrix_len, uint32_t *actual_len, struct bhy2_dev *dev);

/**
 * @brief Function to get the calibration profile of a BSX virtual sensor
 * @param[in] phys_sensor_id: Physical Sensor ID of the virtual sensor
 * @param[out] calib_prof   : Reference to the data buffer to store the calibration profile
 * @param[in] prof_len      : Length of the data buffer
 * @param[out] actual_len   : Actual length of the calibration profile
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_calibration_profile(uint8_t phys_sensor_id,
                                    uint8_t *calib_prof,
                                    uint16_t prof_len,
                                    uint32_t *actual_len,
                                    struct bhy2_dev *dev);

/**
 * @brief Function to set the calibration profile of a BSX virtual sensor
 * @param[in] phys_sensor_id: Physical Sensor ID of the virtual sensor
 * @param[in] calib_prof    : Reference to the data buffer storing the calibration profile
 * @param[in] prof_len      : Length of the profil
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_set_calibration_profile(uint8_t phys_sensor_idv,
                                    const uint8_t *calib_prof,
                                    uint16_t prof_len,
                                    struct bhy2_dev *dev);

/**
 * @brief Function to get the post mortem data
 * @param[out] post_mortem  : Reference to the data buffer to store the post mortem data
 * @param[in] buffer_len    : Length of the data buffer
 * @param[out] actual_len   : Actual length of the post mortem data
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_post_mortem_data(uint8_t *post_mortem, uint32_t buffer_len, uint32_t *actual_len, struct bhy2_dev *dev);

/**
 * @brief Function to link a callback and relevant reference when the sensor event is available in the FIFO
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] callback      : Reference of the callback function
 * @param[in] callback_ref  : Reference needed inside the callback function. Can be NULL
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_register_fifo_parse_callback(uint8_t sensor_id,
                                         bhy2_fifo_parse_callback_t callback,
                                         void *callback_ref,
                                         struct bhy2_dev *dev);

/**
 * @brief Function to unlink a callback and relevant reference
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_deregister_fifo_parse_callback(uint8_t sensor_id, struct bhy2_dev *dev);

/**
 * @brief Function to update the callback table's information
 * @param[in] dev   : Device reference
 * @return  API error codes
 */
int8_t bhy2_update_virtual_sensor_list(struct bhy2_dev *dev);

/**
 * @brief Function to get information of a virtual sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] info     : Reference to the data buffer to store the virtual sensor information
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_get_sensor_info(uint8_t sensor_id, struct bhy2_sensor_info *info, struct bhy2_dev *dev);

/**
 * @brief Function to set a parameter
 * @param[in] param     : Parameter ID
 * @param[in] payload   : Reference to the data buffer storing the parameter's payload
 * @param[in] length    : Length of the payload
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhy2_set_parameter(uint16_t param, const uint8_t *payload, uint32_t length, struct bhy2_dev *dev);

/**
 * @brief Function to get a parameter
 * @param[in] param         : Parameter ID
 * @param[out] payload      : Reference to the data buffer to store the parameter's payload
 * @param[in] payload_len   : Length of the data buffer
 * @param[out] actual_len   : Actual length of the payload
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_parameter(uint16_t param,
                          uint8_t *payload,
                          uint32_t payload_len,
                          uint32_t *actual_len,
                          struct bhy2_dev *dev);

/**
 * @brief Function to get the error value register
 * @param[out] error_value  : Reference to the data buffer to store the error value
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_get_error_value(uint8_t *error_value, struct bhy2_dev *dev);

/**
 * @brief Function to directly communicate with the sensor
 * @param[in] conf          : The configuration of the transfer
 * @param[in] reg_addr      : Register address to write to
 * @param[in] length        : Length of the data buffer to be transferred
 * @param[in/out] reg_data  : Reference to the data buffer
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhy2_soft_passthrough_transfer(union bhy2_soft_passthrough_conf *conf,
                                      uint8_t reg_addr,
                                      uint8_t length,
                                      uint8_t *reg_data,
                                      struct bhy2_dev *dev);

/**
 * @brief Function to check if a virtual sensor is available
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] dev           : Device reference
 * @return 1 if the sensor is available. 0 otherwise
 */
uint8_t bhy2_is_sensor_available(uint8_t sensor_id, const struct bhy2_dev *dev);

/**
 * @brief Function to get the BHy260 variant ID
 *
 * @param[in] variant_id : Reference to store the variant ID
 * @param[in] dev        : Device reference
 * @return int8_t API error codes
 */
int8_t bhy2_get_variant_id(uint32_t *variant_id, struct bhy2_dev *dev);

/**
 * @brief Function to inject data
 * @param[in] payload       : Reference to the data buffer
 * @param[in] payload_len   : Length of the data buffer
 * @param[in] dev           : Device reference
 * @return API error codes
 */

int8_t bhy2_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhy2_dev *dev);

/**
 * @brief Function to set inject mode
 * @param[in] mode       : Type of data inject mode
 * @param[in] dev        : Device reference
 * @return API error codes
 */
int8_t bhy2_set_data_injection_mode(enum bhy2_data_inj_mode mode, struct bhy2_dev *dev);

/**
* @brief Function to clear the FIFO
* @param[in] flush_cfg  : Type of FIFO Flush
* @param[in] dev        : Device reference
* @return API error codes
*/
int8_t bhy2_clear_fifo(uint8_t flush_cfg, struct bhy2_dev *dev);

/**
* @brief Function to read the status FIFO
* @param[out] status_code  : Status Code
* @param[in] status_buff   : Buffer for reading Status Response
* @param[in] status_len    : Length of the Response
* @param[out] actual_len   : Length of Data read
* @param[in] dev           : Device reference
* @return API error codes
*/
int8_t bhy2_read_status(uint16_t *status_code,
                        uint8_t *status_buff,
                        uint32_t status_len,
                        uint32_t *actual_len,
                        struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY2_H__ */
