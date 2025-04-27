/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      SensorBhy2Define.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-30
 *
 */
#pragma once

#include <stdint.h>
#include <stdio.h>

#define BHI260AP_SLAVE_ADDRESS_L          0x28
#define BHI260AP_SLAVE_ADDRESS_H          0x29
#define BHY_PROCESS_BUFFER_SIZE           512

#define BHY2_RLST_CHECK(ret, str, val) \
    do                                 \
    {                                  \
        if (ret)                       \
        {                              \
            log_e(str);                \
            return val;                 \
        }                               \
    } while (0)

typedef void (*BhyEventCb)(uint8_t event, uint8_t *data, uint32_t size);
typedef void (*BhyParseDataCallback)(uint8_t sensor_id, uint8_t *data, uint32_t size,uint64_t *timestamp);



typedef struct SensorEventCbList {
    static uint8_t current_id;
    uint8_t id;
    BhyEventCb cb;
    uint8_t event;
    uint8_t *data;
    SensorEventCbList() : id(current_id++), cb(NULL),  event(0), data(NULL)
    {
    }
} SensorEventCbList_t;


typedef struct ParseCallBackList {
    static uint8_t current_id;
    uint8_t id;
    BhyParseDataCallback cb;
    uint32_t data_length;
    uint8_t *data;
    ParseCallBackList() : id(current_id++), cb(NULL),  data_length(0), data(NULL)
    {
    }
} ParseCallBackList_t;

enum BhySensorEvent {
    BHY2_EVENT_FLUSH_COMPLETE           = 1,
    BHY2_EVENT_SAMPLE_RATE_CHANGED,
    BHY2_EVENT_POWER_MODE_CHANGED,
    BHY2_EVENT_ALGORITHM_EVENTS         = 5,
    BHY2_EVENT_SENSOR_STATUS,
    BHY2_EVENT_BSX_DO_STEPS_MAIN,
    BHY2_EVENT_BSX_DO_STEPS_CALIB,
    BHY2_EVENT_BSX_GET_OUTPUT_SIGNAL,
    BHY2_EVENT_SENSOR_ERROR             = 11,
    BHY2_EVENT_FIFO_OVERFLOW,
    BHY2_EVENT_DYNAMIC_RANGE_CHANGED,
    BHY2_EVENT_FIFO_WATERMARK,
    BHY2_EVENT_INITIALIZED              = 16,
    BHY2_EVENT_TRANSFER_CAUSE,
    BHY2_EVENT_SENSOR_FRAMEWORK,
    BHY2_EVENT_RESET,
};


enum BhySensorID {
    SENSOR_ID_ACC_PASS                 = 1,   /* Accelerometer passthrough */
    SENSOR_ID_ACC_RAW                  = 3,   /* Accelerometer uncalibrated */
    SENSOR_ID_ACC                      = 4,   /* Accelerometer corrected */
    SENSOR_ID_ACC_BIAS                 = 5,   /* Accelerometer offset */
    SENSOR_ID_ACC_WU                   = 6,   /* Accelerometer corrected wake up */
    SENSOR_ID_ACC_RAW_WU               = 7,   /* Accelerometer uncalibrated wake up */
    SENSOR_ID_GYRO_PASS                = 10,  /* Gyroscope passthrough */
    SENSOR_ID_GYRO_RAW                 = 12,  /* Gyroscope uncalibrated */
    SENSOR_ID_GYRO                     = 13,  /* Gyroscope corrected */
    SENSOR_ID_GYRO_BIAS                = 14,  /* Gyroscope offset */
    SENSOR_ID_GYRO_WU                  = 15,  /* Gyroscope wake up */
    SENSOR_ID_GYRO_RAW_WU              = 16,  /* Gyroscope uncalibrated wake up */
    SENSOR_ID_MAG_PASS                 = 19,  /* Magnetometer passthrough */
    SENSOR_ID_MAG_RAW                  = 21,  /* Magnetometer uncalibrated */
    SENSOR_ID_MAG                      = 22,  /* Magnetometer corrected */
    SENSOR_ID_MAG_BIAS                 = 23,  /* Magnetometer offset */
    SENSOR_ID_MAG_WU                   = 24,  /* Magnetometer wake up */
    SENSOR_ID_MAG_RAW_WU               = 25,  /* Magnetometer uncalibrated wake up */
    SENSOR_ID_GRA                      = 28,  /* Gravity vector */
    SENSOR_ID_GRA_WU                   = 29,  /* Gravity vector wake up */
    SENSOR_ID_LACC                     = 31,  /* Linear acceleration */
    SENSOR_ID_LACC_WU                  = 32,  /* Linear acceleration wake up */
    SENSOR_ID_RV                       = 34,  /* Rotation vector */
    SENSOR_ID_RV_WU                    = 35,  /* Rotation vector wake up */
    SENSOR_ID_GAMERV                   = 37,  /* Game rotation vector */
    SENSOR_ID_GAMERV_WU                = 38,  /* Game rotation vector wake up */
    SENSOR_ID_GEORV                    = 40,  /* Geo-magnetic rotation vector */
    SENSOR_ID_GEORV_WU                 = 41,  /* Geo-magnetic rotation vector wake up */
    SENSOR_ID_ORI                      = 43,  /* Orientation */
    SENSOR_ID_ORI_WU                   = 44,  /* Orientation wake up */
    SENSOR_ID_TILT_DETECTOR            = 48,  /* Tilt detector */
    SENSOR_ID_STD                      = 50,  /* Step detector */
    SENSOR_ID_STC                      = 52,  /* Step counter */
    SENSOR_ID_STC_WU                   = 53,  /* Step counter wake up */
    SENSOR_ID_SIG                      = 55,  /* Significant motion */
    SENSOR_ID_WAKE_GESTURE             = 57,  /* Wake gesture */
    SENSOR_ID_GLANCE_GESTURE           = 59,  /* Glance gesture */
    SENSOR_ID_PICKUP_GESTURE           = 61,  /* Pickup gesture */
    SENSOR_ID_AR                       = 63,  /* Activity recognition */
    SENSOR_ID_WRIST_TILT_GESTURE       = 67,  /* Wrist tilt gesture */
    SENSOR_ID_DEVICE_ORI               = 69,  /* Device orientation */
    SENSOR_ID_DEVICE_ORI_WU            = 70,  /* Device orientation wake up */
    SENSOR_ID_STATIONARY_DET           = 75,  /* Stationary detect */
    SENSOR_ID_MOTION_DET               = 77,  /* Motion detect */
    SENSOR_ID_ACC_BIAS_WU              = 91,  /* Accelerometer offset wake up */
    SENSOR_ID_GYRO_BIAS_WU             = 92,  /* Gyroscope offset wake up */
    SENSOR_ID_MAG_BIAS_WU              = 93,  /* Magnetometer offset wake up */
    SENSOR_ID_STD_WU                   = 94,  /* Step detector wake up */
    SENSOR_ID_BSEC                     = 115, /* BSEC 1.x output */
    SENSOR_ID_TEMP                     = 128, /* Temperature */
    SENSOR_ID_BARO                     = 129, /* Barometer */
    SENSOR_ID_HUM                      = 130, /* Humidity */
    SENSOR_ID_GAS                      = 131, /* Gas */
    SENSOR_ID_TEMP_WU                  = 132, /* Temperature wake up */
    SENSOR_ID_BARO_WU                  = 133, /* Barometer wake up */
    SENSOR_ID_HUM_WU                   = 134, /* Humidity wake up */
    SENSOR_ID_GAS_WU                   = 135, /* Gas wake up */
    SENSOR_ID_STC_HW                   = 136, /* Hardware Step counter */
    SENSOR_ID_STD_HW                   = 137, /* Hardware Step detector */
    SENSOR_ID_SIG_HW                   = 138, /* Hardware Significant motion */
    SENSOR_ID_STC_HW_WU                = 139, /* Hardware Step counter wake up */
    SENSOR_ID_STD_HW_WU                = 140, /* Hardware Step detector wake up */
    SENSOR_ID_SIG_HW_WU                = 141, /* Hardware Significant motion wake up */
    SENSOR_ID_ANY_MOTION               = 142, /* Any motion */
    SENSOR_ID_ANY_MOTION_WU            = 143, /* Any motion wake up */
    SENSOR_ID_EXCAMERA                 = 144, /* External camera trigger */
    SENSOR_ID_GPS                      = 145, /* GPS */
    SENSOR_ID_LIGHT                    = 146, /* Light */
    SENSOR_ID_PROX                     = 147, /* Proximity */
    SENSOR_ID_LIGHT_WU                 = 148, /* Light wake up */
    SENSOR_ID_PROX_WU                  = 149, /* Proximity wake up */
    SENSOR_ID_BSEC_LEGACY              = 171, /* BSEC 1.x output (legacy, deprecated) */
    SENSOR_DEBUG_DATA_EVENT            = 250, /* Binary or string debug data */
    SENSOR_TIMESTAMP_SMALL_DELTA       = 251, /* Incremental time change from previous read */
    SENSOR_TIMESTAMP_SMALL_DELTA_WU    = 245, /* Incremental time change from previous read wake up */
    SENSOR_TIMESTAMP_LARGE_DELTA       = 252, /* Incremental time change from previous read */
    SENSOR_TIMESTAMP_LARGE_DELTA_WU    = 246, /* Incremental time change from previous read wake up */
    SENSOR_TIMESTAMP_FULL              = 253, /* Incremental time change from previous read */
    SENSOR_TIMESTAMP_FULL_WU           = 247, /* Incremental time change from previous read wake up */
    SENSOR_ALL_ID                      = 255  /* All Sensor parse event*/
};


#define SENSOR_EULER_ID                 SENSOR_ID_ORI_WU
#define QUAT_SENSOR_ID                  SENSOR_ID_RV





















