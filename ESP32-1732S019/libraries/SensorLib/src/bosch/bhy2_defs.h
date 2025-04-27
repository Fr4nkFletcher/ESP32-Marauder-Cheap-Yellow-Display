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
* @file       bhy2_defs.h
* @date       2023-03-24
* @version    v1.6.0
*
*/

#ifndef __BHY2_DEFS_H__
#define __BHY2_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#ifdef __KERNEL__
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#else
#include <string.h>
#include <stdint.h>
#endif /*~ __KERNEL__ */

#ifndef UNUSED
#define UNUSED(__x__)                             ((void)(__x__))
#endif /*~ UNUSED */

#ifdef __KERNEL__
#define bhy2_float                                u32
#else
#define bhy2_float                                float
#endif /*~ __KERNEL__ */

#ifdef __KERNEL__

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)                                 S8_C(x)
#define UINT8_C(x)                                U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)                                S16_C(x)
#define UINT16_C(x)                               U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)                                S32_C(x)
#define UINT32_C(x)                               U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)                                S64_C(x)
#define UINT64_C(x)                               U64_C(x)
#endif

#else /* __KERNEL__ */

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)                                 (x)
#define UINT8_C(x)                                (x##U)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)                                (x)
#define UINT16_C(x)                               (x##U)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)                                (x)
#define UINT32_C(x)                               (x##U)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)                                (x##LL)
#define UINT64_C(x)                               (x##ULL)
#endif

#endif /* __KERNEL__ */

#ifndef NULL
#ifdef __cplusplus
#define NULL                                      0
#else
#define NULL                                      ((void *) 0)
#endif
#endif

#ifndef BHY2_PACKED
#define BHY2_PACKED                               __attribute__ ((__packed__))
#endif

/** API error codes */
#define BHY2_OK                                   INT8_C(0)
#define BHY2_E_NULL_PTR                           INT8_C(-1)
#define BHY2_E_INVALID_PARAM                      INT8_C(-2)
#define BHY2_E_IO                                 INT8_C(-3)
#define BHY2_E_MAGIC                              INT8_C(-4)
#define BHY2_E_TIMEOUT                            INT8_C(-5)
#define BHY2_E_BUFFER                             INT8_C(-6)
#define BHY2_E_INVALID_FIFO_TYPE                  INT8_C(-7)
#define BHY2_E_INVALID_EVENT_SIZE                 INT8_C(-8)
#define BHY2_E_PARAM_NOT_SET                      INT8_C(-9)
#define BHy2_E_INSUFFICIENT_MAX_SIMUL_SENSORS     INT8_C(-10)

#ifndef BHY2_COMMAND_PACKET_LEN
#define BHY2_COMMAND_PACKET_LEN                   UINT16_C(256)
#endif

#ifndef BHY2_COMMAND_HEADER_LEN
#define BHY2_COMMAND_HEADER_LEN                   UINT16_C(4)
#endif

#if (BHY2_COMMAND_PACKET_LEN < BHY2_COMMAND_HEADER_LEN)
#error "BHY2_COMMAND_PACKET_LEN should be at least 4 bytes"
#endif

#define BHY2_PRODUCT_ID                           UINT8_C(0x89)

/*! Register map */
#define BHY2_REG_CHAN_CMD                         UINT8_C(0x00)
#define BHY2_REG_CHAN_FIFO_W                      UINT8_C(0x01)
#define BHY2_REG_CHAN_FIFO_NW                     UINT8_C(0x02)
#define BHY2_REG_CHAN_STATUS                      UINT8_C(0x03)
#define BHY2_REG_CHIP_CTRL                        UINT8_C(0x05)
#define BHY2_REG_HOST_INTERFACE_CTRL              UINT8_C(0x06)
#define BHY2_REG_HOST_INTERRUPT_CTRL              UINT8_C(0x07)
#define BHY2_REG_RESET_REQ                        UINT8_C(0x14)
#define BHY2_REG_TIME_EV_REQ                      UINT8_C(0x15)
#define BHY2_REG_HOST_CTRL                        UINT8_C(0x16)
#define BHY2_REG_HOST_STATUS                      UINT8_C(0x17)
#define BHY2_REG_CRC_0                            UINT8_C(0x18) /* Totally 4 */
#define BHY2_REG_PRODUCT_ID                       UINT8_C(0x1C)
#define BHY2_REG_REVISION_ID                      UINT8_C(0x1D)
#define BHY2_REG_ROM_VERSION_0                    UINT8_C(0x1E) /* Totally 2 */
#define BHY2_REG_KERNEL_VERSION_0                 UINT8_C(0x20) /* Totally 2 */
#define BHY2_REG_USER_VERSION_0                   UINT8_C(0x22) /* Totally 2 */
#define BHY2_REG_FEATURE_STATUS                   UINT8_C(0x24)
#define BHY2_REG_BOOT_STATUS                      UINT8_C(0x25)
#define BHY2_REG_HOST_INTR_TIME_0                 UINT8_C(0x26) /* Totally 5 */
#define BHY2_REG_CHIP_ID                          UINT8_C(0x2B)
#define BHY2_REG_INT_STATUS                       UINT8_C(0x2D)
#define BHY2_REG_ERROR_VALUE                      UINT8_C(0x2E)
#define BHY2_REG_ERROR_AUX                        UINT8_C(0x2F)
#define BHY2_REG_DEBUG_VALUE                      UINT8_C(0x30)
#define BHY2_REG_DEBUG_STATE                      UINT8_C(0x31)
#define BHY2_REG_GP_5                             UINT8_C(0x32)
#define BHY2_REG_GP_6                             UINT8_C(0x36)
#define BHY2_REG_GP_7                             UINT8_C(0x3A)

/*! Command packets */
#define BHY2_CMD_REQ_POST_MORTEM_DATA             UINT16_C(0x0001)
#define BHY2_CMD_UPLOAD_TO_PROGRAM_RAM            UINT16_C(0x0002)
#define BHY2_CMD_BOOT_PROGRAM_RAM                 UINT16_C(0x0003)
#define BHY2_CMD_ERASE_FLASH                      UINT16_C(0x0004)
#define BHY2_CMD_WRITE_FLASH                      UINT16_C(0x0005)
#define BHY2_CMD_BOOT_FLASH                       UINT16_C(0x0006)
#define BHY2_CMD_SET_INJECT_MODE                  UINT16_C(0x0007)
#define BHY2_CMD_INJECT_DATA                      UINT16_C(0x0008)
#define BHY2_CMD_FIFO_FLUSH                       UINT16_C(0x0009)
#define BHY2_CMD_SW_PASSTHROUGH                   UINT16_C(0x000A)
#define BHY2_CMD_REQ_SELF_TEST                    UINT16_C(0x000B)
#define BHY2_CMD_REQ_FOC                          UINT16_C(0x000C)
#define BHY2_CMD_CONFIG_SENSOR                    UINT16_C(0x000D)
#define BHY2_CMD_CHANGE_RANGE                     UINT16_C(0x000E)
#define BHY2_CMD_FIFO_FORMAT_CTRL                 UINT16_C(0x0015)

/*! Soft passthrough feature */
#define BHY2_SPASS_READ                           UINT8_C(0)
#define BHY2_SPASS_WRITE                          UINT8_C(1)
#define BHY2_SPASS_SINGLE_TRANS                   UINT8_C(0)
#define BHY2_SPASS_MULTI_TRANS                    UINT8_C(1)
#define BHY2_SPASS_DELAY_DIS                      UINT8_C(0)
#define BHY2_SPASS_DELAY_EN                       UINT8_C(1)
#define BHY2_SPASS_SIF1                           UINT8_C(1)
#define BHY2_SPASS_SIF2                           UINT8_C(2)
#define BHY2_SPASS_SIF3                           UINT8_C(3)
#define BHY2_SPASS_SPI_4_WIRE                     UINT8_C(0)
#define BHY2_SPASS_SPI_3_WIRE                     UINT8_C(1)
#define BHY2_SPASS_SPI_CPOL_0                     UINT8_C(0)
#define BHY2_SPASS_SPI_CPOL_1                     UINT8_C(1)
#define BHY2_SPASS_SPI_CPHA_0                     UINT8_C(0)
#define BHY2_SPASS_SPI_CPHA_1                     UINT8_C(1)
#define BHY2_SPASS_SPI_CS_LOW                     UINT8_C(0)
#define BHY2_SPASS_SPI_CS_HIGH                    UINT8_C(1)
#define BHY2_SPASS_SPI_LSB_FIRST_DIS              UINT8_C(0)
#define BHY2_SPASS_SPI_LSB_FIRST_EN               UINT8_C(1)
#define BHY2_SPASS_SPI_READ_BIT_POL_LOW           UINT8_C(0)
#define BHY2_SPASS_SPI_READ_BIT_POL_HIGH          UINT8_C(1)
#define BHY2_SPASS_SPI_READ_BIT_POS_0             UINT8_C(0)
#define BHY2_SPASS_SPI_READ_BIT_POS_1             UINT8_C(1)
#define BHY2_SPASS_SPI_READ_BIT_POS_2             UINT8_C(2)
#define BHY2_SPASS_SPI_READ_BIT_POS_3             UINT8_C(3)
#define BHY2_SPASS_SPI_READ_BIT_POS_4             UINT8_C(4)
#define BHY2_SPASS_SPI_READ_BIT_POS_5             UINT8_C(5)
#define BHY2_SPASS_SPI_READ_BIT_POS_6             UINT8_C(6)
#define BHY2_SPASS_SPI_READ_BIT_POS_7             UINT8_C(7)
#define BHY2_SPASS_READ_PACKET_LEN                UINT8_C(0x13)
#define BHY2_SPASS_WRITE_RESP_PACKET_LEN          UINT8_C(16)

/*! Helper macros */
#define BHY2_CHK_BIT(data, bit)                   (((uint32_t)data >> bit) & 0x01)
#define BHY2_ROUND_WORD_LOWER(x)                  ((x >> 2) << 2)
#define BHY2_ROUND_WORD_HIGHER(x)                 (((x >> 2) + 1) << 2)

#define BHY2_COUNTOF(__BUFFER__)                  (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define BHY2_DATA_INJECT_MODE_PAYLOAD_LEN         UINT8_C(0x04)

/*! Firmware header identifier */
#define BHY2_FW_MAGIC                             UINT16_C(0x662B)

/*! BHy260 Variant IDs */
#define BHY2_VARIANT_ID_BHA260AB                  UINT32_C(0x2838C2C2)
#define BHY2_VARIANT_ID_BHI260AB                  UINT32_C(0x266EB6E7)
#define BHY2_VARIANT_ID_BHI260AP                  UINT32_C(0x98D7A5D1)

/*! Boot status */
#define BHY2_BST_FLASH_DETECTED                   UINT8_C(0x01)
#define BHY2_BST_FLASH_VERIFY_DONE                UINT8_C(0x02)
#define BHY2_BST_FLASH_VERIFY_ERROR               UINT8_C(0x04)
#define BHY2_BST_NO_FLASH                         UINT8_C(0x08)
#define BHY2_BST_HOST_INTERFACE_READY             UINT8_C(0x10)
#define BHY2_BST_HOST_FW_VERIFY_DONE              UINT8_C(0x20)
#define BHY2_BST_HOST_FW_VERIFY_ERROR             UINT8_C(0x40)
#define BHY2_BST_HOST_FW_IDLE                     UINT8_C(0x80)
#define BHY2_BST_CHECK_RETRY                      UINT8_C(100)

/*! Host status */
#define BHY2_HST_POWER_STATE                      UINT8_C(0x01)
#define BHY2_HST_HOST_PROTOCOL                    UINT8_C(0x02)
#define BHY2_HST_HOST_CHANNEL_0                   UINT8_C(0x10)
#define BHY2_HST_HOST_CHANNEL_1                   UINT8_C(0x20)
#define BHY2_HST_HOST_CHANNEL_2                   UINT8_C(0x40)
#define BHY2_HST_HOST_CHANNEL_3                   UINT8_C(0x80)

/*! Interrupt status masks */
#define BHY2_IST_MASK_ASSERTED                    (0x1)
#define BHY2_IST_MASK_FIFO_W                      (0x6)
#define BHY2_IST_MASK_FIFO_NW                     (0x18)
#define BHY2_IST_MASK_STATUS                      (0x20)
#define BHY2_IST_MASK_DEBUG                       (0x40)
#define BHY2_IST_MASK_RESET_FAULT                 (0x80)
#define BHY2_IST_FIFO_W_DRDY                      UINT8_C(0x2)
#define BHY2_IST_FIFO_W_LTCY                      UINT8_C(0x4)
#define BHY2_IST_FIFO_W_WM                        UINT8_C(0x6)
#define BHY2_IST_FIFO_NW_DRDY                     UINT8_C(0x8)
#define BHY2_IST_FIFO_NW_LTCY                     UINT8_C(0x10)
#define BHY2_IST_FIFO_NW_WM                       UINT8_C(0x18)
#define BHY2_IS_INT_FIFO_W(status)                (status & BHY2_IST_MASK_FIFO_W)
#define BHY2_IS_INT_FIFO_NW(status)               (status & BHY2_IST_MASK_FIFO_NW)
#define BHY2_IS_INT_STATUS(status)                (status & BHY2_IST_MASK_STATUS)
#define BHY2_IS_INT_ASYNC_STATUS(status)          (status & BHY2_IST_MASK_DEBUG)
#define BHY2_IS_INT_RESET(status)                 (status & BHY2_IST_MASK_RESET_FAULT)
#define BHY2_IST_MASK_FIFO                        (BHY2_IST_MASK_FIFO_W | BHY2_IST_MASK_FIFO_NW)
#define BHY2_IS_INT_FIFO(status)                  (status & BHY2_IST_MASK_FIFO)

/*! Chip control bits */
#define BHY2_CHIP_CTRL_DISABLE                    UINT8_C(0x00)
#define BHY2_CHIP_CTRL_TURBO_ENABLE               UINT8_C(0x01)
#define BHY2_CHIP_CTRL_CLR_ERR_REG                UINT8_C(0x02)

/*! Host interface control bits */
#define BHY2_HIF_CTRL_ABORT_TRANSFER_CHANNEL_0    UINT8_C(0x01)
#define BHY2_HIF_CTRL_ABORT_TRANSFER_CHANNEL_1    UINT8_C(0x02)
#define BHY2_HIF_CTRL_ABORT_TRANSFER_CHANNEL_2    UINT8_C(0x04)
#define BHY2_HIF_CTRL_ABORT_TRANSFER_CHANNEL_3    UINT8_C(0x08)
#define BHY2_HIF_CTRL_AP_SUSPENDED                UINT8_C(0x10)
#define BHY2_HIF_CTRL_TIMESTAMP_EV_CTRL           UINT8_C(0x40)
#define BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL        UINT8_C(0x80)

/*! Interrupt control bits */
#define BHY2_ICTL_DISABLE_FIFO_W                  UINT8_C(0x01)
#define BHY2_ICTL_DISABLE_FIFO_NW                 UINT8_C(0x02)
#define BHY2_ICTL_DISABLE_STATUS_FIFO             UINT8_C(0x04)
#define BHY2_ICTL_DISABLE_DEBUG                   UINT8_C(0x08)
#define BHY2_ICTL_DISABLE_FAULT                   UINT8_C(0x10)
#define BHY2_ICTL_ACTIVE_LOW                      UINT8_C(0x20)
#define BHY2_ICTL_EDGE                            UINT8_C(0x40)
#define BHY2_ICTL_OPEN_DRAIN                      UINT8_C(0x80)

/*! Reset command */
#define BHY2_REQUEST_RESET                        UINT8_C(0x01)

/*! FIFO Format bit */
#define BHY2_FIFO_FORMAT_CTRL_MASK                UINT8_C(0x03)
#define BHY2_FIFO_FORMAT_CTRL_DIS_DELTA_TS        UINT8_C(0x01)
#define BHY2_FIFO_FORMAT_CTRL_DIS_FULL_TS         UINT8_C(0x02)
#define BHY2_FIFO_FORMAT_CTRL_DIS_ALL_TS          UINT8_C(0x03)

/*! System parameters */
#define BHY2_PARAM_READ_MASK                      UINT16_C(0x1000)
#define BHY2_PARAM_FIFO_CTRL                      UINT16_C(0x103)
#define BHY2_PARAM_SYS_VIRT_SENSOR_PRESENT        UINT16_C(0x11F)
#define BHY2_PARAM_SYS_PHYS_SENSOR_PRESENT        UINT16_C(0x120)
#define BHY2_PARAM_PHYSICAL_SENSOR_BASE           UINT16_C(0x120)
#define BHY2_PARAM_BSX_CALIB_STATE_BASE           UINT16_C(0x200)
#define BHY2_PARAM_BSX_CALIB_STATE_ACCEL          UINT16_C(0x201)
#define BHY2_PARAM_BSX_CALIB_STATE_GYRO           UINT16_C(0x203)
#define BHY2_PARAM_BSX_CALIB_STATE_MAG            UINT16_C(0x205)
#define BHY2_PARAM_BSX_SIC                        UINT16_C(0x27D)
#define BHY2_PARAM_BSX_VERSION                    UINT16_C(0x27E)
#define BHY2_PARAM_SET_SENSOR_CTRL                UINT16_C(0x0E00)
#define BHY2_PARAM_GET_SENSOR_CTRL                UINT16_C(0x1E00)
#define BHY2_PARAM_SENSOR_CTRL_FOC                UINT8_C(0x1)
#define BHY2_PARAM_SENSOR_CTRL_OIS                UINT8_C(0x2)
#define BHY2_PARAM_SENSOR_CTRL_FST                UINT8_C(0x3)
#define BHY2_PARAM_SENSOR_CTRL_READ               UINT8_C(0x80)
#define BHY2_PARAM_SENSOR_INFO_0                  UINT16_C(0x300)
#define BHY2_PARAM_SENSOR_CONF_0                  UINT16_C(0x500)

#define BHY2_BSX_STATE_TRANSFER_COMPLETE          UINT8_C(0x80)
#define BHY2_BSX_STATE_BLOCK_NUM_MSK              UINT8_C(0x7F)
#define BHY2_BSX_STATE_BLOCK_LEN                  UINT8_C(64)
#define BHY2_BSX_STATE_STRUCT_LEN                 UINT8_C(68)

#define BHY2_QUERY_PARAM_STATUS_READY_MAX_RETRY   UINT16_C(1000)
#define BHY2_QUERY_FLASH_MAX_RETRY                UINT16_C(1200)

/*! Meta event definitions */
#define BHY2_META_EVENT_FLUSH_COMPLETE            (1)
#define BHY2_META_EVENT_SAMPLE_RATE_CHANGED       (2)
#define BHY2_META_EVENT_POWER_MODE_CHANGED        (3)
#define BHY2_META_EVENT_ALGORITHM_EVENTS          (5)
#define BHY2_META_EVENT_SENSOR_STATUS             (6)
#define BHY2_META_EVENT_BSX_DO_STEPS_MAIN         (7)
#define BHY2_META_EVENT_BSX_DO_STEPS_CALIB        (8)
#define BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL     (9)
#define BHY2_META_EVENT_RESERVED1                 (10)
#define BHY2_META_EVENT_SENSOR_ERROR              (11)
#define BHY2_META_EVENT_FIFO_OVERFLOW             (12)
#define BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED     (13)
#define BHY2_META_EVENT_FIFO_WATERMARK            (14)
#define BHY2_META_EVENT_RESERVED2                 (15)
#define BHY2_META_EVENT_INITIALIZED               (16)
#define BHY2_META_TRANSFER_CAUSE                  (17)
#define BHY2_META_EVENT_SENSOR_FRAMEWORK          (18)
#define BHY2_META_EVENT_RESET                     (19)
#define BHY2_META_EVENT_SPACER                    (20)

/* Flash start address in memory */
#define BHY2_FLASH_SECTOR_START_ADDR              UINT32_C(0x1F84)
#define BHY2_FLASH_BULK_ERASE                     UINT32_C(0xFFFFFFFF)

/* End address for common flash sizes */
#define BHY2_FLASH_SIZE_0_5MB                     UINT32_C(0x7FFFF)
#define BHY2_FLASH_SIZE_1MB                       UINT32_C(0xFFFFF)
#define BHY2_FLASH_SIZE_2MB                       UINT32_C(0x1FFFFF)
#define BHY2_FLASH_SIZE_4MB                       UINT32_C(0x3FFFFF)
#define BHY2_FLASH_SIZE_8MB                       UINT32_C(0x7FFFFF)

/* Sensor IDs */
#define BHY2_SENSOR_ID_CUSTOM_START               UINT8_C(160)
#define BHY2_SENSOR_ID_CUSTOM_END                 UINT8_C(191)

#define BHY2_SENSOR_ID_MAX                        UINT8_C(200)
#define BHY2_SENSOR_ID_TBD                        UINT8_C(BHY2_SENSOR_ID_MAX - 1)

/* Virtual Sensor IDs */
#define BHY2_SENSOR_ID_ACC_PASS                   UINT8_C(1) /* Accelerometer passthrough */
#define BHY2_SENSOR_ID_ACC_RAW                    UINT8_C(3) /* Accelerometer uncalibrated */
#define BHY2_SENSOR_ID_ACC                        UINT8_C(4) /* Accelerometer corrected */
#define BHY2_SENSOR_ID_ACC_BIAS                   UINT8_C(5) /* Accelerometer offset */
#define BHY2_SENSOR_ID_ACC_WU                     UINT8_C(6) /* Accelerometer corrected wake up */
#define BHY2_SENSOR_ID_ACC_RAW_WU                 UINT8_C(7) /* Accelerometer uncalibrated wake up */
#define BHY2_SENSOR_ID_SI_ACCEL                   UINT8_C(8) /* Virtual Sensor ID for Accelerometer */
#define BHY2_SENSOR_ID_GYRO_PASS                  UINT8_C(10) /* Gyroscope passthrough */
#define BHY2_SENSOR_ID_GYRO_RAW                   UINT8_C(12) /* Gyroscope uncalibrated */
#define BHY2_SENSOR_ID_GYRO                       UINT8_C(13) /* Gyroscope corrected */
#define BHY2_SENSOR_ID_GYRO_BIAS                  UINT8_C(14) /* Gyroscope offset */
#define BHY2_SENSOR_ID_GYRO_WU                    UINT8_C(15) /* Gyroscope wake up */
#define BHY2_SENSOR_ID_GYRO_RAW_WU                UINT8_C(16) /* Gyroscope uncalibrated wake up */
#define BHY2_SENSOR_ID_SI_GYROS                   UINT8_C(17) /* Virtual Sensor ID for Gyroscope */
#define BHY2_SENSOR_ID_MAG_PASS                   UINT8_C(19) /* Magnetometer passthrough */
#define BHY2_SENSOR_ID_MAG_RAW                    UINT8_C(21) /* Magnetometer uncalibrated */
#define BHY2_SENSOR_ID_MAG                        UINT8_C(22) /* Magnetometer corrected */
#define BHY2_SENSOR_ID_MAG_BIAS                   UINT8_C(23) /* Magnetometer offset */
#define BHY2_SENSOR_ID_MAG_WU                     UINT8_C(24) /* Magnetometer wake up */
#define BHY2_SENSOR_ID_MAG_RAW_WU                 UINT8_C(25) /* Magnetometer uncalibrated wake up */
#define BHY2_SENSOR_ID_GRA                        UINT8_C(28) /* Gravity vector */
#define BHY2_SENSOR_ID_GRA_WU                     UINT8_C(29) /* Gravity vector wake up */
#define BHY2_SENSOR_ID_LACC                       UINT8_C(31) /* Linear acceleration */
#define BHY2_SENSOR_ID_LACC_WU                    UINT8_C(32) /* Linear acceleration wake up */
#define BHY2_SENSOR_ID_RV                         UINT8_C(34) /* Rotation vector */
#define BHY2_SENSOR_ID_RV_WU                      UINT8_C(35) /* Rotation vector wake up */
#define BHY2_SENSOR_ID_GAMERV                     UINT8_C(37) /* Game rotation vector */
#define BHY2_SENSOR_ID_GAMERV_WU                  UINT8_C(38) /* Game rotation vector wake up */
#define BHY2_SENSOR_ID_GEORV                      UINT8_C(40) /* Geo-magnetic rotation vector */
#define BHY2_SENSOR_ID_GEORV_WU                   UINT8_C(41) /* Geo-magnetic rotation vector wake up */
#define BHY2_SENSOR_ID_ORI                        UINT8_C(43) /* Orientation */
#define BHY2_SENSOR_ID_ORI_WU                     UINT8_C(44) /* Orientation wake up */
#define BHY2_SENSOR_ID_TILT_DETECTOR              UINT8_C(48) /* Tilt detector */
#define BHY2_SENSOR_ID_STD                        UINT8_C(50) /* Step detector */
#define BHY2_SENSOR_ID_STC                        UINT8_C(52) /* Step counter */
#define BHY2_SENSOR_ID_STC_WU                     UINT8_C(53) /* Step counter wake up */
#define BHY2_SENSOR_ID_SIG                        UINT8_C(55) /* Significant motion */
#define BHY2_SENSOR_ID_WAKE_GESTURE               UINT8_C(57) /* Wake gesture */
#define BHY2_SENSOR_ID_GLANCE_GESTURE             UINT8_C(59) /* Glance gesture */
#define BHY2_SENSOR_ID_PICKUP_GESTURE             UINT8_C(61) /* Pickup gesture */
#define BHY2_SENSOR_ID_AR                         UINT8_C(63) /* Activity recognition */
#define BHY2_SENSOR_ID_WRIST_TILT_GESTURE         UINT8_C(67) /* Wrist tilt gesture */
#define BHY2_SENSOR_ID_DEVICE_ORI                 UINT8_C(69) /* Device orientation */
#define BHY2_SENSOR_ID_DEVICE_ORI_WU              UINT8_C(70) /* Device orientation wake up */
#define BHY2_SENSOR_ID_STATIONARY_DET             UINT8_C(75) /* Stationary detect */
#define BHY2_SENSOR_ID_MOTION_DET                 UINT8_C(77) /* Motion detect */
#define BHY2_SENSOR_ID_ACC_BIAS_WU                UINT8_C(91) /* Accelerometer offset wake up */
#define BHY2_SENSOR_ID_GYRO_BIAS_WU               UINT8_C(92) /* Gyroscope offset wake up */
#define BHY2_SENSOR_ID_MAG_BIAS_WU                UINT8_C(93) /* Magnetometer offset wake up */
#define BHY2_SENSOR_ID_STD_WU                     UINT8_C(94) /* Step detector wake up */
#define BHY2_SENSOR_ID_TEMP                       UINT8_C(128) /* Temperature */
#define BHY2_SENSOR_ID_BARO                       UINT8_C(129) /* Barometer */
#define BHY2_SENSOR_ID_HUM                        UINT8_C(130) /* Humidity */
#define BHY2_SENSOR_ID_GAS                        UINT8_C(131) /* Gas */
#define BHY2_SENSOR_ID_TEMP_WU                    UINT8_C(132) /* Temperature wake up */
#define BHY2_SENSOR_ID_BARO_WU                    UINT8_C(133) /* Barometer wake up */
#define BHY2_SENSOR_ID_HUM_WU                     UINT8_C(134) /* Humidity wake up */
#define BHY2_SENSOR_ID_GAS_WU                     UINT8_C(135) /* Gas wake up */
#define BHY2_SENSOR_ID_STC_LP                     UINT8_C(136) /* Step counter Low Power */
#define BHY2_SENSOR_ID_STD_LP                     UINT8_C(137) /* Step detector Low Power */
#define BHY2_SENSOR_ID_SIG_LP                     UINT8_C(138) /* Significant motion Low Power */
#define BHY2_SENSOR_ID_STC_LP_WU                  UINT8_C(139) /* Step counter Low Power wake up */
#define BHY2_SENSOR_ID_STD_LP_WU                  UINT8_C(140) /* Step detector Low Power wake up */
#define BHY2_SENSOR_ID_SIG_LP_WU                  UINT8_C(141) /* Significant motion Low Power wake up */
#define BHY2_SENSOR_ID_ANY_MOTION_LP              UINT8_C(142) /* Any motion Low Power */
#define BHY2_SENSOR_ID_ANY_MOTION_LP_WU           UINT8_C(143) /* Any motion Low Power wake up */
#define BHY2_SENSOR_ID_EXCAMERA                   UINT8_C(144) /* External camera trigger */
#define BHY2_SENSOR_ID_GPS                        UINT8_C(145) /* GPS */
#define BHY2_SENSOR_ID_LIGHT                      UINT8_C(146) /* Light */
#define BHY2_SENSOR_ID_PROX                       UINT8_C(147) /* Proximity */
#define BHY2_SENSOR_ID_LIGHT_WU                   UINT8_C(148) /* Light wake up */
#define BHY2_SENSOR_ID_PROX_WU                    UINT8_C(149) /* Proximity wake up */

/*! Physical sensor IDs*/
#define BHY2_PHYS_SENSOR_ID_ACCELEROMETER         UINT8_C(1)
#define BHY2_PHYS_SENSOR_ID_GYROSCOPE             UINT8_C(3)
#define BHY2_PHYS_SENSOR_ID_MAGNETOMETER          UINT8_C(5)
#define BHY2_PHYS_SENSOR_ID_TEMP_GYRO             UINT8_C(7)
#define BHY2_PHYS_SENSOR_ID_ANY_MOTION            UINT8_C(9)
#define BHY2_PHYS_SENSOR_ID_PRESSURE              UINT8_C(11)
#define BHY2_PHYS_SENSOR_ID_POSITION              UINT8_C(13)
#define BHY2_PHYS_SENSOR_ID_HUMIDITY              UINT8_C(15)
#define BHY2_PHYS_SENSOR_ID_TEMPERATURE           UINT8_C(17)
#define BHY2_PHYS_SENSOR_ID_GAS_RESISTOR          UINT8_C(19)
#define BHY2_PHYS_SENSOR_ID_PHYS_STEP_COUNTER     UINT8_C(32)
#define BHY2_PHYS_SENSOR_ID_PHYS_STEP_DETECTOR    UINT8_C(33)
#define BHY2_PHYS_SENSOR_ID_PHYS_SIGN_MOTION      UINT8_C(34)
#define BHY2_PHYS_SENSOR_ID_PHYS_ANY_MOTION       UINT8_C(35)
#define BHY2_PHYS_SENSOR_ID_EX_CAMERA_INPUT       UINT8_C(36)
#define BHY2_PHYS_SENSOR_ID_GPS                   UINT8_C(48)
#define BHY2_PHYS_SENSOR_ID_LIGHT                 UINT8_C(49)
#define BHY2_PHYS_SENSOR_ID_PROXIMITY             UINT8_C(50)
#define BHY2_PHYS_SENSOR_ID_PHYS_NO_MOTION        UINT8_C(55)
#define BHY2_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT  UINT8_C(56)
#define BHY2_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP     UINT8_C(57)

/*! System data IDs */
#define BHY2_IS_SYS_ID(sid)                       ((sid) >= 224)

#define BHY2_SYS_ID_PADDING                       UINT8_C(0)
#define BHY2_SYS_ID_TS_SMALL_DELTA                UINT8_C(251)
#define BHY2_SYS_ID_TS_LARGE_DELTA                UINT8_C(252)
#define BHY2_SYS_ID_TS_FULL                       UINT8_C(253)
#define BHY2_SYS_ID_META_EVENT                    UINT8_C(254)
#define BHY2_SYS_ID_TS_SMALL_DELTA_WU             UINT8_C(245)
#define BHY2_SYS_ID_TS_LARGE_DELTA_WU             UINT8_C(246)
#define BHY2_SYS_ID_TS_FULL_WU                    UINT8_C(247)
#define BHY2_SYS_ID_META_EVENT_WU                 UINT8_C(248)
#define BHY2_SYS_ID_FILLER                        UINT8_C(255)
#define BHY2_SYS_ID_DEBUG_MSG                     UINT8_C(250)
#define BHY2_SYS_ID_BHY2_LOG_UPDATE_SUB           UINT8_C(243)
#define BHY2_SYS_ID_BHY2_LOG_DOSTEP               UINT8_C(244)

/*! Status code definitions */
#define BHY2_STATUS_INITIALIZED                   UINT8_C(0x1)
#define BHY2_STATUS_DEBUG_OUTPUT                  UINT8_C(0x2)
#define BHY2_STATUS_CRASH_DUMP                    UINT8_C(0x3)
#define BHY2_STATUS_INJECT_SENSOR_CONF_REQ        UINT8_C(0x4)
#define BHY2_STATUS_SW_PASS_THRU_RES              UINT8_C(0x5)
#define BHY2_STATUS_SELF_TEST_RES                 UINT8_C(0x6)
#define BHY2_STATUS_FOC_RES                       UINT8_C(0x7)
#define BHY2_STATUS_SYSTEM_ERROR                  UINT8_C(0x8)
#define BHY2_STATUS_SENSOR_ERROR                  UINT8_C(0x9)
#define BHY2_STATUS_FLASH_ERASE_COMPLETE          UINT8_C(0xA)
#define BHY2_STATUS_FLASH_WRITE_COMPLETE          UINT8_C(0xB)
#define BHY2_STATUS_FLASH_CONTINUE_UPLOAD         UINT8_C(0xC)
#define BHY2_STATUS_HOST_EV_TIMESTAMP             UINT8_C(0xD)
#define BHY2_STATUS_DUT_TEST_RES                  UINT8_C(0xE)
#define BHY2_STATUS_CMD_ERR                       UINT8_C(0xF)

#define BHY2_IS_STATUS_GET_PARAM_OUTPUT(status)   ((status) >= 0x100 && (status) <= 0xFFF)

/*! Activity bits */
#define BHY2_STILL_ACTIVITY_ENDED                 (0x0001)
#define BHY2_WALKING_ACTIVITY_ENDED               (0x0002)
#define BHY2_RUNNING_ACTIVITY_ENDED               (0x0004)
#define BHY2_ON_BICYCLE_ACTIVITY_ENDED            (0x0008)
#define BHY2_IN_VEHICLE_ACTIVITY_ENDED            (0x0010)
#define BHY2_TILTING_ACTIVITY_ENDED               (0x0020)

#define BHY2_STILL_ACTIVITY_STARTED               (0x0100)
#define BHY2_WALKING_ACTIVITY_STARTED             (0x0200)
#define BHY2_RUNNING_ACTIVITY_STARTED             (0x0400)
#define BHY2_ON_BICYCLE_ACTIVITY_STARTED          (0x0800)
#define BHY2_IN_VEHICLE_ACTIVITY_STARTED          (0x1000)
#define BHY2_TILTING_ACTIVITY_STARTED             (0x2000)

/*! Feature status */
#define BHY2_FEAT_STATUS_FLASH_DESC_MSK           UINT8_C(0x01)
#define BHY2_FEAT_STATUS_OPEN_RTOS_MSK            UINT8_C(0x02)
#define BHY2_FEAT_STATUS_OPEN_RTOS_POS            (1)
#define BHY2_FEAT_STATUS_HOST_ID_MSK              UINT8_C(0x1C)
#define BHY2_FEAT_STATUS_HOST_ID_POS              (2)
#define BHY2_FEAT_STATUS_ALGO_ID_MSK              UINT8_C(0xE0)
#define BHY2_FEAT_STATUS_ALGO_ID_POS              (5)

/*! Fast offset compensation status codes */
#define BHY2_FOC_PASS                             UINT8_C(0x00)
#define BHY2_FOC_FAILED                           UINT8_C(0x65)
#define BHY2_FOC_UNKNOWN_FAILURE                  UINT8_C(0x23)

/*! Self test status codes */
#define BHY2_ST_PASSED                            UINT8_C(0x00)
#define BHY2_ST_X_AXIS_FAILED                     UINT8_C(0x01)
#define BHY2_ST_Y_AXIS_FAILED                     UINT8_C(0x02)
#define BHY2_ST_Z_AXIS_FAILED                     UINT8_C(0x04)
#define BHY2_ST_MULTI_AXIS_FAILURE                UINT8_C(0x07)
#define BHY2_ST_NOT_SUPPORTED                     UINT8_C(0x08)
#define BHY2_ST_INVALID_PHYS_ID                   UINT8_C(0x09)

#define BHY2_LE2U16(x)                            ((uint16_t)((x)[0] | (x)[1] << 8))
#define BHY2_LE2S16(x)                            ((int16_t)BHY2_LE2U16(x))
#define BHY2_LE2U24(x)                            ((uint32_t)((x)[0] | (uint32_t)(x)[1] << 8 | (uint32_t)(x)[2] << 16))
#define BHY2_LE2S24(x)                            ((int32_t)(BHY2_LE2U24(x) << 8) >> 8)
#define BHY2_LE2U32(x)                            ((uint32_t)((x)[0] | (uint32_t)(x)[1] << 8 | (uint32_t)(x)[2] << 16 | \
                                                              (uint32_t)(x)[3] << 24))
#define BHY2_LE2S32(x)                            ((int32_t)BHY2_LE2U32(x))
#define BHY2_LE2U40(x)                            (BHY2_LE2U32(x) | (uint64_t)(x)[4] << 32)
#define BHY2_LE2U64(x)                            (BHY2_LE2U32(x) | (uint64_t)BHY2_LE2U32(&(x)[4]) << 32)

#define BHY2_LE24MUL(x)                           (((x) % 4) ? (uint16_t)((((x) / 4) + 1) * 4) : (uint16_t)((x) + 4))

/*! Maximum no of available virtual sensor */
#define BHY2_N_VIRTUAL_SENSOR_MAX                 UINT8_C(256)

#ifndef BHY2_MAX_SIMUL_SENSORS
#define BHY2_MAX_SIMUL_SENSORS                    48
#endif

/* Special & debug virtual sensor id starts at 245 */
#define BHY2_SPECIAL_SENSOR_ID_OFFSET             UINT8_C(245)

#ifndef BHY2_INTF_RET_TYPE
#define BHY2_INTF_RET_TYPE                        int8_t
#endif

#ifndef BHY2_INTF_RET_SUCCESS
#define BHY2_INTF_RET_SUCCESS                     0
#endif

/* Type definitions for the function pointers */
typedef BHY2_INTF_RET_TYPE (*bhy2_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
typedef BHY2_INTF_RET_TYPE (*bhy2_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,
        void *intf_ptr);
typedef void (*bhy2_delay_us_fptr_t)(uint32_t period_us, void *intf_ptr);

enum bhy2_intf {
    BHY2_SPI_INTERFACE = 1,
    BHY2_I2C_INTERFACE
};

/* HIF device structure */
struct bhy2_hif_dev {
    bhy2_read_fptr_t read;
    bhy2_write_fptr_t write;
    bhy2_delay_us_fptr_t delay_us;
    enum bhy2_intf intf;
    void *intf_ptr;
    BHY2_INTF_RET_TYPE intf_rslt;
    uint32_t read_write_len;
};

enum bhy2_fifo_type {
    BHY2_FIFO_TYPE_WAKEUP,
    BHY2_FIFO_TYPE_NON_WAKEUP,
    BHY2_FIFO_TYPE_STATUS,
    BHY2_FIFO_TYPE_MAX
};

struct BHY2_PACKED bhy2_fifo_parse_data_info {
    uint8_t sensor_id;
    enum bhy2_fifo_type fifo_type;
    uint8_t data_size;
    uint8_t *data_ptr;
    uint64_t *time_stamp;
};

typedef void (*bhy2_fifo_parse_callback_t)(const struct bhy2_fifo_parse_data_info *callback_info, void *private_data);

struct BHY2_PACKED bhy2_fifo_parse_callback_table {
    uint8_t sensor_id;
    bhy2_fifo_parse_callback_t callback;
    void *callback_ref;
};

/* Device structure */
struct bhy2_dev {
    struct bhy2_hif_dev hif;
    struct bhy2_fifo_parse_callback_table table[BHY2_MAX_SIMUL_SENSORS];
    uint8_t event_size[BHY2_N_VIRTUAL_SENSOR_MAX];
    uint64_t last_time_stamp[BHY2_FIFO_TYPE_MAX];
    uint8_t present_buff[32];
};

struct bhy2_fifo_buffer {
    uint32_t read_pos;
    uint32_t read_length;
    uint32_t remain_length;
    uint32_t buffer_size;
    uint8_t *buffer;
};

typedef int16_t (*bhy2_frame_parse_func_t)(struct bhy2_fifo_buffer *p_fifo_buffer, struct bhy2_dev *bhy2_p);

struct bhy2_virt_sensor_conf {
    uint16_t sensitivity;
    uint16_t range;
    uint32_t latency;
    bhy2_float sample_rate;
};

struct bhy2_data_xyz {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct bhy2_data_quaternion {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    uint16_t accuracy;
};

struct bhy2_data_orientation {
    int16_t heading;
    int16_t pitch;
    int16_t roll;
};

union BHY2_PACKED bhy2_spt_dev_id {
    uint8_t slave_address;
    uint8_t cs_pin;
};

struct BHY2_PACKED bhy2_spt_bits {
    /*! byte 1 */
    uint8_t direction : 1; /**< 0: read; 1: write. */
    uint8_t trans_type : 1; /**< 0: single burst; 1:multi single transfers. */
    uint8_t delay_ctrl : 1; /**< 0: none; 1: delay between bytes. */
    uint8_t master_bus : 2; /**< 1: SIF1; 2: SIF2; 3:SIF3. */
    uint8_t spi_mode : 1; /**< 0: 4 wire; 1: 3 wire. */
    uint8_t cpol : 1; /**< spi clock polarity. */
    uint8_t cpha : 1; /**< spi clock phase. */
    /*! byte 2 */
    uint8_t delay_val : 6; /**< multiples of 50us. min is 200us (4LSB) */
    uint8_t cs_level : 1; /**< chip select level. */
    uint8_t lsb_first : 1; /**< least significant byte first. */
    /*! byte 3~4 */
    uint16_t trans_rate; /**< spi clock rate. */
    /*! byte 5 */
    uint8_t address_shift : 4; /**< number of bits to shift register address. */
    uint8_t read_bit_pol : 1; /**< 0: active low; 1: active high. */
    uint8_t read_bit_pos : 3; /**< bit number of read bit in command byte. */
    /*! byte 6 */
    union bhy2_spt_dev_id func_set;

    /*! byte 7 */
    uint8_t trans_count;

    /*! byte 8 */
    uint8_t reg;
};

union bhy2_soft_passthrough_conf {
    struct bhy2_spt_bits conf;
    uint8_t data[8];
};

union bhy2_u16_conv {
    uint16_t u16_val;
    uint8_t bytes[2];
};
union bhy2_u32_conv {
    uint32_t u32_val;
    uint8_t bytes[4];
};
union bhy2_float_conv {
    bhy2_float f_val;
    uint32_t u32_val;
    uint8_t bytes[4];
};
struct bhy2_sensor_info {
    uint8_t sensor_type;
    uint8_t driver_id;
    uint8_t driver_version;
    uint8_t power;
    union bhy2_u16_conv max_range;
    union bhy2_u16_conv resolution;
    union bhy2_float_conv max_rate;
    union bhy2_u32_conv fifo_reserved;
    union bhy2_u32_conv fifo_max;
    uint8_t event_size;
    union bhy2_float_conv min_rate;
};

struct bhy2_phys_sensor_info {
    uint8_t sensor_type;
    uint8_t driver_id;
    uint8_t driver_version;
    uint8_t power_current;
    union bhy2_u16_conv curr_range;
    uint8_t flags;
    uint8_t slave_address;
    uint8_t gpio_assignment;
    union bhy2_float_conv curr_rate;
    uint8_t num_axis;
    uint8_t orientation_matrix[5];
    uint8_t reserved;
};

struct bhy2_self_test_resp {
    uint8_t test_status;
    int16_t x_offset, y_offset, z_offset;
};

struct bhy2_foc_resp {
    uint8_t foc_status;
    int16_t x_offset, y_offset, z_offset;
};

struct bhy2_orient_matrix {
    int8_t c[9];
};

enum bhy2_data_inj_mode {
    BHY2_NORMAL_MODE = 0,
    BHY2_REAL_TIME_INJECTION = 1,
    BHY2_STEP_BY_STEP_INJECTION = 2
};

#define BHY2_BYTE_TO_NIBBLE(X)  (((uint8_t)(X)[0] & 0x0F) | (((uint8_t)(X)[1] << 4) & 0xF0))

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHY2_DEFS_H__ */
