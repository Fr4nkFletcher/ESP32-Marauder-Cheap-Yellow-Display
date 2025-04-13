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
 * @file    common.c
 * @brief   Common source file for the BHy260 examples
 * @note    Adapting Arduino based on BHY2-Sensor-API by lewis
 */

#include "common.h"
#include "bosch/bhy2_parse.h"
#include "bosch/bhy2.h"


void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

const char *get_api_error(int8_t error_code)
{
    const char *ret;

    switch (error_code) {
    case BHY2_OK:
        ret = "[API Success]";
        break;
    case BHY2_E_NULL_PTR:
        ret = "[API Error] Null pointer";
        break;
    case BHY2_E_INVALID_PARAM:
        ret = "[API Error] Invalid parameter";
        break;
    case BHY2_E_IO:
        ret = "[API Error] IO error";
        break;
    case BHY2_E_MAGIC:
        ret = "[API Error] Invalid firmware";
        break;
    case BHY2_E_TIMEOUT:
        ret = "[API Error] Timed out";
        break;
    case BHY2_E_BUFFER:
        ret = "[API Error] Invalid buffer";
        break;
    case BHY2_E_INVALID_FIFO_TYPE:
        ret = "[API Error] Invalid FIFO type";
        break;
    case BHY2_E_INVALID_EVENT_SIZE:
        ret = "[API Error] Invalid Event size";
        break;
    case BHY2_E_PARAM_NOT_SET:
        ret = "[API Error] Parameter not set";
        break;
    default:
        ret = "[API Error] Unknown API error code";
    }

    return ret;
}

const char *get_sensor_error_text(uint8_t sensor_error)
{
    const char *ret;

    switch (sensor_error) {
    case 0x00:
        break;
    case 0x10:
        ret = "[Sensor error] Bootloader reports: Firmware Expected Version Mismatch";
        break;
    case 0x11:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Header CRC";
        break;
    case 0x12:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch";
        break;
    case 0x13:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Image CRC";
        break;
    case 0x14:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed";
        break;
    case 0x15:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Public Key CRC";
        break;
    case 0x16:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed Firmware Required";
        break;
    case 0x17:
        ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing";
        break;
    case 0x19:
        ret = "[Sensor error] Bootloader reports: Unexpected Watchdog Reset";
        break;
    case 0x1A:
        ret = "[Sensor error] ROM Version Mismatch";
        break;
    case 0x1B:
        ret = "[Sensor error] Bootloader reports: Fatal Firmware Error";
        break;
    case 0x1C:
        ret = "[Sensor error] Chained Firmware Error: Next Payload Not Found";
        break;
    case 0x1D:
        ret = "[Sensor error] Chained Firmware Error: Payload Not Valid";
        break;
    case 0x1E:
        ret = "[Sensor error] Chained Firmware Error: Payload Entries Invalid";
        break;
    case 0x1F:
        ret = "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC Invalid";
        break;
    case 0x20:
        ret = "[Sensor error] Firmware Init Failed";
        break;
    case 0x21:
        ret = "[Sensor error] Sensor Init Failed: Unexpected Device ID";
        break;
    case 0x22:
        ret = "[Sensor error] Sensor Init Failed: No Response from Device";
        break;
    case 0x23:
        ret = "[Sensor error] Sensor Init Failed: Unknown";
        break;
    case 0x24:
        ret = "[Sensor error] Sensor Error: No Valid Data";
        break;
    case 0x25:
        ret = "[Sensor error] Slow Sample Rate";
        break;
    case 0x26:
        ret = "[Sensor error] Data Overflow (saturated sensor data)";
        break;
    case 0x27:
        ret = "[Sensor error] Stack Overflow";
        break;
    case 0x28:
        ret = "[Sensor error] Insufficient Free RAM";
        break;
    case 0x29:
        ret = "[Sensor error] Sensor Init Failed: Driver Parsing Error";
        break;
    case 0x2A:
        ret = "[Sensor error] Too Many RAM Banks Required";
        break;
    case 0x2B:
        ret = "[Sensor error] Invalid Event Specified";
        break;
    case 0x2C:
        ret = "[Sensor error] More than 32 On Change";
        break;
    case 0x2D:
        ret = "[Sensor error] Firmware Too Large";
        break;
    case 0x2F:
        ret = "[Sensor error] Invalid RAM Banks";
        break;
    case 0x30:
        ret = "[Sensor error] Math Error";
        break;
    case 0x40:
        ret = "[Sensor error] Memory Error";
        break;
    case 0x41:
        ret = "[Sensor error] SWI3 Error";
        break;
    case 0x42:
        ret = "[Sensor error] SWI4 Error";
        break;
    case 0x43:
        ret = "[Sensor error] Illegal Instruction Error";
        break;
    case 0x44:
        ret = "[Sensor error] Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available";
        break;
    case 0x45:
        ret = "[Sensor error] Invalid Memory Access";
        break;
    case 0x50:
        ret = "[Sensor error] Algorithm Error: BSX Init";
        break;
    case 0x51:
        ret = "[Sensor error] Algorithm Error: BSX Do Step";
        break;
    case 0x52:
        ret = "[Sensor error] Algorithm Error: Update Sub";
        break;
    case 0x53:
        ret = "[Sensor error] Algorithm Error: Get Sub";
        break;
    case 0x54:
        ret = "[Sensor error] Algorithm Error: Get Phys";
        break;
    case 0x55:
        ret = "[Sensor error] Algorithm Error: Unsupported Phys Rate";
        break;
    case 0x56:
        ret = "[Sensor error] Algorithm Error: Cannot find BSX Driver";
        break;
    case 0x60:
        ret = "[Sensor error] Sensor Self-Test Failure";
        break;
    case 0x61:
        ret = "[Sensor error] Sensor Self-Test X Axis Failure";
        break;
    case 0x62:
        ret = "[Sensor error] Sensor Self-Test Y Axis Failure";
        break;
    case 0x64:
        ret = "[Sensor error] Sensor Self-Test Z Axis Failure";
        break;
    case 0x65:
        ret = "[Sensor error] FOC Failure";
        break;
    case 0x66:
        ret = "[Sensor error] Sensor Busy";
        break;
    case 0x6F:
        ret = "[Sensor error] Self-Test or FOC Test Unsupported";
        break;
    case 0x72:
        ret = "[Sensor error] No Host Interrupt Set";
        break;
    case 0x73:
        ret = "[Sensor error] Event ID Passed to Host Interface Has No Known Size";
        break;
    case 0x75:
        ret = "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)";
        break;
    case 0x76:
        ret = "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)";
        break;
    case 0x77:
        ret = "[Sensor error] Host Download Channel Empty";
        break;
    case 0x78:
        ret = "[Sensor error] DMA Error";
        break;
    case 0x79:
        ret = "[Sensor error] Corrupted Input Block Chain";
        break;
    case 0x7A:
        ret = "[Sensor error] Corrupted Output Block Chain";
        break;
    case 0x7B:
        ret = "[Sensor error] Buffer Block Manager Error";
        break;
    case 0x7C:
        ret = "[Sensor error] Input Channel Not Word Aligned";
        break;
    case 0x7D:
        ret = "[Sensor error] Too Many Flush Events";
        break;
    case 0x7E:
        ret = "[Sensor error] Unknown Host Channel Error";
        break;
    case 0x81:
        ret = "[Sensor error] Decimation Too Large";
        break;
    case 0x90:
        ret = "[Sensor error] Master SPI/I2C Queue Overflow";
        break;
    case 0x91:
        ret = "[Sensor error] SPI/I2C Callback Error";
        break;
    case 0xA0:
        ret = "[Sensor error] Timer Scheduling Error";
        break;
    case 0xB0:
        ret = "[Sensor error] Invalid GPIO for Host IRQ";
        break;
    case 0xB1:
        ret = "[Sensor error] Error Sending Initialized Meta Events";
        break;
    case 0xC0:
        ret = "[Sensor error] Bootloader reports: Command Error";
        break;
    case 0xC1:
        ret = "[Sensor error] Bootloader reports: Command Too Long";
        break;
    case 0xC2:
        ret = "[Sensor error] Bootloader reports: Command Buffer Overflow";
        break;
    case 0xD0:
        ret = "[Sensor error] User Mode Error: Sys Call Invalid";
        break;
    case 0xD1:
        ret = "[Sensor error] User Mode Error: Trap Invalid";
        break;
    case 0xE1:
        ret = "[Sensor error] Firmware Upload Failed: Firmware header corrupt";
        break;
    case 0xE2:
        ret = "[Sensor error] Sensor Data Injection: Invalid input stream";
        break;
    default:
        ret = "[Sensor error] Unknown error code";
    }

    return ret;
}

const char *get_sensor_name(uint8_t sensor_id)
{
    const char *ret;

    switch (sensor_id) {
    case BHY2_SENSOR_ID_ACC_PASS:
        ret = "Accelerometer passthrough";
        break;
    case BHY2_SENSOR_ID_ACC_RAW:
        ret = "Accelerometer uncalibrated";
        break;
    case BHY2_SENSOR_ID_ACC:
        ret = "Accelerometer corrected";
        break;
    case BHY2_SENSOR_ID_ACC_BIAS:
        ret = "Accelerometer offset";
        break;
    case BHY2_SENSOR_ID_ACC_WU:
        ret = "Accelerometer corrected wake up";
        break;
    case BHY2_SENSOR_ID_ACC_RAW_WU:
        ret = "Accelerometer uncalibrated wake up";
        break;
    case BHY2_SENSOR_ID_GYRO_PASS:
        ret = "Gyroscope passthrough";
        break;
    case BHY2_SENSOR_ID_GYRO_RAW:
        ret = "Gyroscope uncalibrated";
        break;
    case BHY2_SENSOR_ID_GYRO:
        ret = "Gyroscope corrected";
        break;
    case BHY2_SENSOR_ID_GYRO_BIAS:
        ret = "Gyroscope offset";
        break;
    case BHY2_SENSOR_ID_GYRO_WU:
        ret = "Gyroscope wake up";
        break;
    case BHY2_SENSOR_ID_GYRO_RAW_WU:
        ret = "Gyroscope uncalibrated wake up";
        break;
    case BHY2_SENSOR_ID_MAG_PASS:
        ret = "Magnetometer passthrough";
        break;
    case BHY2_SENSOR_ID_MAG_RAW:
        ret = "Magnetometer uncalibrated";
        break;
    case BHY2_SENSOR_ID_MAG:
        ret = "Magnetometer corrected";
        break;
    case BHY2_SENSOR_ID_MAG_BIAS:
        ret = "Magnetometer offset";
        break;
    case BHY2_SENSOR_ID_MAG_WU:
        ret = "Magnetometer wake up";
        break;
    case BHY2_SENSOR_ID_MAG_RAW_WU:
        ret = "Magnetometer uncalibrated wake up";
        break;
    case BHY2_SENSOR_ID_GRA:
        ret = "Gravity vector";
        break;
    case BHY2_SENSOR_ID_GRA_WU:
        ret = "Gravity vector wake up";
        break;
    case BHY2_SENSOR_ID_LACC:
        ret = "Linear acceleration";
        break;
    case BHY2_SENSOR_ID_LACC_WU:
        ret = "Linear acceleration wake up";
        break;
    case BHY2_SENSOR_ID_RV:
        ret = "Rotation vector";
        break;
    case BHY2_SENSOR_ID_RV_WU:
        ret = "Rotation vector wake up";
        break;
    case BHY2_SENSOR_ID_GAMERV:
        ret = "Game rotation vector";
        break;
    case BHY2_SENSOR_ID_GAMERV_WU:
        ret = "Game rotation vector wake up";
        break;
    case BHY2_SENSOR_ID_GEORV:
        ret = "Geo-magnetic rotation vector";
        break;
    case BHY2_SENSOR_ID_GEORV_WU:
        ret = "Geo-magnetic rotation vector wake up";
        break;
    case BHY2_SENSOR_ID_ORI:
        ret = "Orientation";
        break;
    case BHY2_SENSOR_ID_ORI_WU:
        ret = "Orientation wake up";
        break;
    case BHY2_SENSOR_ID_ACC_BIAS_WU:
        ret = "Accelerometer offset wake up";
        break;
    case BHY2_SENSOR_ID_GYRO_BIAS_WU:
        ret = "Gyroscope offset wake up";
        break;
    case BHY2_SENSOR_ID_MAG_BIAS_WU:
        ret = "Magnetometer offset wake up";
        break;
    case BHY2_SENSOR_ID_TEMP:
        ret = "Temperature";
        break;
    case BHY2_SENSOR_ID_BARO:
        ret = "Barometer";
        break;
    case BHY2_SENSOR_ID_HUM:
        ret = "Humidity";
        break;
    case BHY2_SENSOR_ID_GAS:
        ret = "Gas";
        break;
    case BHY2_SENSOR_ID_TEMP_WU:
        ret = "Temperature wake up";
        break;
    case BHY2_SENSOR_ID_BARO_WU:
        ret = "Barometer wake up";
        break;
    case BHY2_SENSOR_ID_HUM_WU:
        ret = "Humidity wake up";
        break;
    case BHY2_SENSOR_ID_GAS_WU:
        ret = "Gas wake up";
        break;
    case BHY2_SENSOR_ID_KLIO:
        ret = "Klio";
        break;
    case BHY2_SENSOR_ID_KLIO_LOG:
        ret = "Klio log";
        break;
    case BHY2_SENSOR_ID_SWIM:
        ret = "Swim recognition";
        break;
    case BHY2_SENSOR_ID_SI_ACCEL:
        ret = "SI Accel";
        break;
    case BHY2_SENSOR_ID_SI_GYROS:
        ret = "SI Gyro";
        break;
    case BHY2_SENSOR_ID_LIGHT:
        ret = "Light";
        break;
    case BHY2_SENSOR_ID_LIGHT_WU:
        ret = "Light wake up";
        break;
    case BHY2_SENSOR_ID_PROX:
        ret = "Proximity";
        break;
    case BHY2_SENSOR_ID_PROX_WU:
        ret = "Proximity wake up";
        break;
    case BHY2_SENSOR_ID_STC:
        ret = "Step counter";
        break;
    case BHY2_SENSOR_ID_STC_WU:
        ret = "Step counter wake up";
        break;
    case BHY2_SENSOR_ID_STC_LP:
        ret = "Low Power Step counter";
        break;
    case BHY2_SENSOR_ID_STC_LP_WU:
        ret = "Low Power Step counter wake up";
        break;
    case BHY2_SENSOR_ID_SIG:
        ret = "Significant motion";
        break;
    case BHY2_SENSOR_ID_STD:
        ret = "Step detector";
        break;
    case BHY2_SENSOR_ID_STD_WU:
        ret = "Step detector wake up";
        break;
    case BHY2_SENSOR_ID_TILT_DETECTOR:
        ret = "Tilt detector";
        break;
    case BHY2_SENSOR_ID_WAKE_GESTURE:
        ret = "Wake gesture";
        break;
    case BHY2_SENSOR_ID_GLANCE_GESTURE:
        ret = "Glance gesture";
        break;
    case BHY2_SENSOR_ID_PICKUP_GESTURE:
        ret = "Pickup gesture";
        break;
    case BHY2_SENSOR_ID_SIG_LP:
        ret = "Low Power Significant motion";
        break;
    case BHY2_SENSOR_ID_SIG_LP_WU:
        ret = "Low Power Significant motion wake up";
        break;
    case BHY2_SENSOR_ID_STD_LP:
        ret = "Low Power Step detector";
        break;
    case BHY2_SENSOR_ID_STD_LP_WU:
        ret = "Low Power Step detector wake up";
        break;
    case BHY2_SENSOR_ID_AR:
        ret = "Activity recognition";
        break;
    case BHY2_SENSOR_ID_EXCAMERA:
        ret = "External camera trigger";
        break;
    case BHY2_SENSOR_ID_GPS:
        ret = "GPS";
        break;
    case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
        ret = "Wrist tilt gesture";
        break;
    case BHY2_SENSOR_ID_DEVICE_ORI:
        ret = "Device orientation";
        break;
    case BHY2_SENSOR_ID_DEVICE_ORI_WU:
        ret = "Device orientation wake up";
        break;
    case BHY2_SENSOR_ID_STATIONARY_DET:
        ret = "Stationary detect";
        break;
    case BHY2_SENSOR_ID_ANY_MOTION_LP:
        ret = "Low Power Any motion";
        break;
    case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
        ret = "Low Power Any motion wake up";
        break;
    case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
        ret = "Low Power No Motion wake up";
        break;
    case BHY2_SENSOR_ID_MOTION_DET:
        ret = "Motion detect";
        break;
    case BHI3_SENSOR_ID_AR_WEAR_WU:
        ret = "Activity recognition for Wearables";
        break;
    case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
        ret = "Low Power Wrist Wear wake up";
        break;
    case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
        ret = "Low Power Wrist Gesture wake up";
        break;
    case BHI3_SENSOR_ID_MULTI_TAP:
        ret = "Multi Tap Detector";
        break;
    case BHY2_SENSOR_ID_AIR_QUALITY:
        ret = "Air Quality";
        break;
    case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
        ret = "Head Misalignment Calibrator";
        break;
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
        ret = "IMU Head Orientation Quaternion";
        break;
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
        ret = "NDOF Head Orientation Quaternion";
        break;
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
        ret = "IMU Head Orientation Euler";
        break;
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
        ret = "NDOF Head Orientation Euler";
        break;
    default:
        if ((sensor_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHY2_SENSOR_ID_CUSTOM_END)) {
            ret = "Custom sensor ID ";
        } else {
            ret = "Undefined sensor ID ";
        }
    }

    return ret;
}

float get_sensor_default_scaling(uint8_t sensor_id)
{
    float scaling = -1.0f;

    switch (sensor_id) {
    case BHY2_SENSOR_ID_ACC_PASS:
    case BHY2_SENSOR_ID_ACC_RAW:
    case BHY2_SENSOR_ID_ACC:
    case BHY2_SENSOR_ID_ACC_BIAS:
    case BHY2_SENSOR_ID_ACC_WU:
    case BHY2_SENSOR_ID_ACC_RAW_WU:
    case BHY2_SENSOR_ID_GRA:
    case BHY2_SENSOR_ID_GRA_WU:
    case BHY2_SENSOR_ID_LACC:
    case BHY2_SENSOR_ID_LACC_WU:
    case BHY2_SENSOR_ID_ACC_BIAS_WU:
        scaling = 1.0f / 4096.0f;
        break;
    case BHY2_SENSOR_ID_GYRO_PASS:
    case BHY2_SENSOR_ID_GYRO_RAW:
    case BHY2_SENSOR_ID_GYRO:
    case BHY2_SENSOR_ID_GYRO_BIAS:
    case BHY2_SENSOR_ID_GYRO_WU:
    case BHY2_SENSOR_ID_GYRO_RAW_WU:
    case BHY2_SENSOR_ID_GYRO_BIAS_WU:
        scaling = 2000.0f / 32768.0f;
        break;
    case BHY2_SENSOR_ID_MAG_PASS:
    case BHY2_SENSOR_ID_MAG_RAW:
    case BHY2_SENSOR_ID_MAG:
    case BHY2_SENSOR_ID_MAG_BIAS:
    case BHY2_SENSOR_ID_MAG_WU:
    case BHY2_SENSOR_ID_MAG_RAW_WU:
    case BHY2_SENSOR_ID_MAG_BIAS_WU:
        scaling = 2500.0f / 32768.0f;
        break;
    case BHY2_SENSOR_ID_RV:
    case BHY2_SENSOR_ID_RV_WU:
    case BHY2_SENSOR_ID_GAMERV:
    case BHY2_SENSOR_ID_GAMERV_WU:
    case BHY2_SENSOR_ID_GEORV:
    case BHY2_SENSOR_ID_GEORV_WU:
        scaling = 1.0f / 16384.0f;
        break;
    case BHY2_SENSOR_ID_ORI:
    case BHY2_SENSOR_ID_ORI_WU:
        scaling = 360.0f / 32768.0f;
        break;
    case BHY2_SENSOR_ID_TEMP:
    case BHY2_SENSOR_ID_TEMP_WU:
        scaling = 1.0f / 100.0f;
        break;
    case BHY2_SENSOR_ID_BARO:
    case BHY2_SENSOR_ID_BARO_WU:
        scaling = 100.0f / 128.0f;
        break;
    case BHY2_SENSOR_ID_HUM:
    case BHY2_SENSOR_ID_HUM_WU:
        scaling = 1.0f;
        break;
    case BHY2_SENSOR_ID_GAS:
    case BHY2_SENSOR_ID_GAS_WU:
        scaling = 1.0f;
        break;
    case BHY2_SENSOR_ID_LIGHT:
    case BHY2_SENSOR_ID_LIGHT_WU:
        scaling = 10000.0f / 216.0f;
        break;
    case BHY2_SENSOR_ID_PROX:
    case BHY2_SENSOR_ID_PROX_WU:
        scaling = 1.0f;
        break;
    case BHY2_SENSOR_ID_SI_ACCEL:
    case BHY2_SENSOR_ID_SI_GYROS:

        /* Scaling factor already applied in firmware*/
        break;
    case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
        scaling = 1.0f / 16384.0f; /*2^14 -> 16384*/
        break;
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
        scaling = 360.0f / 32768.0f; /*2^15 -> 32768*/
        break;
    default:
        scaling = -1.0f; /* Do not apply the scaling factor */
    }

    return scaling;
}

const char *get_sensor_parse_format(uint8_t sensor_id)
{
    const char *ret;

    switch (sensor_id) {
    case BHY2_SENSOR_ID_ACC_PASS:
    case BHY2_SENSOR_ID_ACC_RAW:
    case BHY2_SENSOR_ID_ACC:
    case BHY2_SENSOR_ID_ACC_BIAS:
    case BHY2_SENSOR_ID_ACC_BIAS_WU:
    case BHY2_SENSOR_ID_ACC_WU:
    case BHY2_SENSOR_ID_ACC_RAW_WU:
    case BHY2_SENSOR_ID_GYRO_PASS:
    case BHY2_SENSOR_ID_GYRO_RAW:
    case BHY2_SENSOR_ID_GYRO:
    case BHY2_SENSOR_ID_GYRO_BIAS:
    case BHY2_SENSOR_ID_GYRO_BIAS_WU:
    case BHY2_SENSOR_ID_GYRO_WU:
    case BHY2_SENSOR_ID_GYRO_RAW_WU:
    case BHY2_SENSOR_ID_MAG_PASS:
    case BHY2_SENSOR_ID_MAG_RAW:
    case BHY2_SENSOR_ID_MAG:
    case BHY2_SENSOR_ID_MAG_BIAS:
    case BHY2_SENSOR_ID_MAG_BIAS_WU:
    case BHY2_SENSOR_ID_MAG_WU:
    case BHY2_SENSOR_ID_MAG_RAW_WU:
    case BHY2_SENSOR_ID_GRA:
    case BHY2_SENSOR_ID_GRA_WU:
    case BHY2_SENSOR_ID_LACC:
    case BHY2_SENSOR_ID_LACC_WU:
        ret = "s16,s16,s16";
        break;
    case BHY2_SENSOR_ID_RV:
    case BHY2_SENSOR_ID_RV_WU:
    case BHY2_SENSOR_ID_GAMERV:
    case BHY2_SENSOR_ID_GAMERV_WU:
    case BHY2_SENSOR_ID_GEORV:
    case BHY2_SENSOR_ID_GEORV_WU:
        ret = "s16,s16,s16,s16,u16";
        break;
    case BHY2_SENSOR_ID_ORI:
    case BHY2_SENSOR_ID_ORI_WU:
        ret = "s16,s16,s16";
        break;
    case BHY2_SENSOR_ID_DEVICE_ORI:
    case BHY2_SENSOR_ID_DEVICE_ORI_WU:
    case BHY2_SENSOR_ID_HUM:
    case BHY2_SENSOR_ID_HUM_WU:
    case BHY2_SENSOR_ID_PROX:
    case BHY2_SENSOR_ID_PROX_WU:
    case BHY2_SENSOR_ID_EXCAMERA:
    case BHI3_SENSOR_ID_MULTI_TAP:
        ret = "u8";
        break;
    case BHY2_SENSOR_ID_TEMP:
    case BHY2_SENSOR_ID_TEMP_WU:
        ret = "s16";
        break;
    case BHY2_SENSOR_ID_BARO:
    case BHY2_SENSOR_ID_BARO_WU:
        ret = "u24";
        break;
    case BHY2_SENSOR_ID_GAS:
    case BHY2_SENSOR_ID_GAS_WU:
    case BHY2_SENSOR_ID_STC:
    case BHY2_SENSOR_ID_STC_WU:
    case BHY2_SENSOR_ID_STC_LP:
    case BHY2_SENSOR_ID_STC_LP_WU:
        ret = "u32";
        break;
    case BHY2_SENSOR_ID_KLIO:
        ret = "u8,u8,u8,u8,u8,u8,f";
        break;
    case BHY2_SENSOR_ID_SWIM:
        ret = "u16,u16,u16,u16,u16,u16,u16";
        break;
    case BHY2_SENSOR_ID_SI_ACCEL:
    case BHY2_SENSOR_ID_SI_GYROS:
        ret = "f,f,f";
        break;
    case BHY2_SENSOR_ID_LIGHT:
    case BHY2_SENSOR_ID_LIGHT_WU:
        ret = "s16";
        break;
    case BHY2_SENSOR_ID_SIG:
    case BHY2_SENSOR_ID_STD:
    case BHY2_SENSOR_ID_STD_WU:
    case BHY2_SENSOR_ID_TILT_DETECTOR:
    case BHY2_SENSOR_ID_WAKE_GESTURE:
    case BHY2_SENSOR_ID_GLANCE_GESTURE:
    case BHY2_SENSOR_ID_PICKUP_GESTURE:
    case BHY2_SENSOR_ID_SIG_LP:
    case BHY2_SENSOR_ID_SIG_LP_WU:
    case BHY2_SENSOR_ID_STD_LP:
    case BHY2_SENSOR_ID_STD_LP_WU:
    case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
    case BHY2_SENSOR_ID_STATIONARY_DET:
    case BHY2_SENSOR_ID_ANY_MOTION_LP:
    case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
    case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
    case BHY2_SENSOR_ID_MOTION_DET:
    case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
        ret = "";
        break;
    case BHY2_SENSOR_ID_AR:
    case BHI3_SENSOR_ID_AR_WEAR_WU:
        ret = "u16";
        break;
    case BHY2_SENSOR_ID_GPS:
        ret = "st";
        break;
    case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
        ret = "u8";
        break;
    case BHY2_SENSOR_ID_AIR_QUALITY:
        ret = "f32,f32,f32,f32,f32,f32,f32,u8";
        break;
    case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
        ret = "s16,s16,s16,s16,u8";
        break;

    case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
        ret = "s16,s16,s16,u8";
        break;
    default:
        ret = "";
    }

    return ret;
}

const char *get_sensor_axis_names(uint8_t sensor_id)
{
    const char *ret;

    switch (sensor_id) {
    case BHY2_SENSOR_ID_ACC_PASS:
    case BHY2_SENSOR_ID_ACC_RAW:
    case BHY2_SENSOR_ID_ACC:
    case BHY2_SENSOR_ID_ACC_BIAS:
    case BHY2_SENSOR_ID_ACC_BIAS_WU:
    case BHY2_SENSOR_ID_ACC_WU:
    case BHY2_SENSOR_ID_ACC_RAW_WU:
    case BHY2_SENSOR_ID_GYRO_PASS:
    case BHY2_SENSOR_ID_GYRO_RAW:
    case BHY2_SENSOR_ID_GYRO:
    case BHY2_SENSOR_ID_GYRO_BIAS:
    case BHY2_SENSOR_ID_GYRO_BIAS_WU:
    case BHY2_SENSOR_ID_GYRO_WU:
    case BHY2_SENSOR_ID_GYRO_RAW_WU:
    case BHY2_SENSOR_ID_MAG_PASS:
    case BHY2_SENSOR_ID_MAG_RAW:
    case BHY2_SENSOR_ID_MAG:
    case BHY2_SENSOR_ID_MAG_BIAS:
    case BHY2_SENSOR_ID_MAG_BIAS_WU:
    case BHY2_SENSOR_ID_MAG_WU:
    case BHY2_SENSOR_ID_MAG_RAW_WU:
    case BHY2_SENSOR_ID_GRA:
    case BHY2_SENSOR_ID_GRA_WU:
    case BHY2_SENSOR_ID_LACC:
    case BHY2_SENSOR_ID_LACC_WU:
    case BHY2_SENSOR_ID_SI_ACCEL:
    case BHY2_SENSOR_ID_SI_GYROS:
        ret = "x,y,z";
        break;
    case BHY2_SENSOR_ID_RV:
    case BHY2_SENSOR_ID_RV_WU:
    case BHY2_SENSOR_ID_GAMERV:
    case BHY2_SENSOR_ID_GAMERV_WU:
    case BHY2_SENSOR_ID_GEORV:
    case BHY2_SENSOR_ID_GEORV_WU:
        ret = "x,y,z,w,ar";
        break;
    case BHY2_SENSOR_ID_ORI:
    case BHY2_SENSOR_ID_ORI_WU:
        ret = "h,p,r";
        break;
    case BHY2_SENSOR_ID_DEVICE_ORI:
    case BHY2_SENSOR_ID_DEVICE_ORI_WU:
        ret = "o";
        break;
    case BHY2_SENSOR_ID_TEMP:
    case BHY2_SENSOR_ID_TEMP_WU:
        ret = "t";
        break;
    case BHY2_SENSOR_ID_BARO:
    case BHY2_SENSOR_ID_BARO_WU:
        ret = "p";
        break;
    case BHY2_SENSOR_ID_HUM:
    case BHY2_SENSOR_ID_HUM_WU:
        ret = "h";
        break;
    case BHY2_SENSOR_ID_GAS:
    case BHY2_SENSOR_ID_GAS_WU:
        ret = "g";
        break;
    case BHY2_SENSOR_ID_KLIO:
        ret = "lin,lid,lpr,lcr,rin,rid,rc";
        break;
    case BHY2_SENSOR_ID_SWIM:
        ret = "d,lc,f,br,bu,ba,sc";
        break;
    case BHY2_SENSOR_ID_LIGHT:
    case BHY2_SENSOR_ID_LIGHT_WU:
        ret = "l";
        break;
    case BHY2_SENSOR_ID_PROX:
    case BHY2_SENSOR_ID_PROX_WU:
        ret = "p";
        break;
    case BHY2_SENSOR_ID_STC:
    case BHY2_SENSOR_ID_STC_WU:
    case BHY2_SENSOR_ID_STC_LP:
    case BHY2_SENSOR_ID_STC_LP_WU:
    case BHY2_SENSOR_ID_EXCAMERA:
        ret = "c";
        break;
    case BHY2_SENSOR_ID_SIG:
    case BHY2_SENSOR_ID_STD:
    case BHY2_SENSOR_ID_STD_WU:
    case BHY2_SENSOR_ID_TILT_DETECTOR:
    case BHY2_SENSOR_ID_WAKE_GESTURE:
    case BHY2_SENSOR_ID_GLANCE_GESTURE:
    case BHY2_SENSOR_ID_PICKUP_GESTURE:
    case BHY2_SENSOR_ID_SIG_LP:
    case BHY2_SENSOR_ID_SIG_LP_WU:
    case BHY2_SENSOR_ID_STD_LP:
    case BHY2_SENSOR_ID_STD_LP_WU:
    case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
    case BHY2_SENSOR_ID_STATIONARY_DET:
    case BHY2_SENSOR_ID_ANY_MOTION_LP:
    case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
    case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
    case BHY2_SENSOR_ID_MOTION_DET:
    case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
        ret = "e";
        break;
    case BHY2_SENSOR_ID_AR:
    case BHI3_SENSOR_ID_AR_WEAR_WU:
        ret = "a";
        break;
    case BHY2_SENSOR_ID_GPS:
        ret = "g";
        break;
    case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
        ret = "wrist_gesture";
        break;
    case BHI3_SENSOR_ID_MULTI_TAP:
        ret = "taps";
        break;
    case BHY2_SENSOR_ID_AIR_QUALITY:
        ret = "t,h,g,i,si,c,v,a";
        break;
    case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
        ret = "x,y,z,w,a";
        break;
    case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
    case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
        ret = "h,p,r,a";
        break;
    default:
        ret = "";
    }

    return ret;
}

void check_bhy2_api(unsigned int line, const char *func, int8_t val)
{
    int i = 3;
    while (i--) {

        Serial.printf("BHI260 API failed at line %u. The function %s returned error code %d. %s\r\n",
                      line,
                      func,
                      val,
                      get_api_error(val));
        delay(1000);
    }
}

