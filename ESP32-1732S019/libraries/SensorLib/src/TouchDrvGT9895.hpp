/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      TouchDrvGT9895.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#pragma once

#include "REG/GT9895Constants.h"
#include "TouchDrvInterface.hpp"
#include "SensorCommon.tpp"


enum CHECKSUM_MODE {
	CHECKSUM_MODE_U8_LE,
	CHECKSUM_MODE_U16_LE,
};

enum touch_point_status {
    TS_NONE,
    TS_RELEASE,
    TS_TOUCH,
};

/* interrupt event type */
enum ts_event_type {
    EVENT_INVALID   = 0,
    EVENT_TOUCH     = (1 << 0),     /* finger touch event */
    EVENT_PEN       = (1 << 1),     /* pen event */
    EVENT_REQUEST   = (1 << 2),
    EVENT_GESTURE   = (1 << 3),
};

enum brl_request_code {
    BRL_REQUEST_CODE_CONFIG = 0x01,
    BRL_REQUEST_CODE_REF_ERR = 0x02,
    BRL_REQUEST_CODE_RESET = 0x03,
    BRL_REQUEST_CODE_CLOCK = 0x04,
};

/* coordinate package */
struct goodix_ts_coords {
    int status; /* NONE, RELEASE, TOUCH */
    unsigned int x, y, w, p;
};

/* touch event data */
struct goodix_touch_data {
    int touch_num;
    uint64_t timestamp;
    struct goodix_ts_coords coords[GT9895_MAX_TOUCH];
};

struct goodix_ts_key {
    int status;
    int code;
};
struct goodix_pen_coords {
	int status; /* NONE, RELEASE, TOUCH */
	int tool_type;  /* BTN_TOOL_RUBBER BTN_TOOL_PEN */
	unsigned int x, y, p;
	signed char tilt_x;
	signed char tilt_y;
};

struct goodix_pen_data {
    struct goodix_pen_coords coords;
    struct goodix_ts_key keys[GT9895_MAX_PEN_KEY];
};

struct goodix_point_t {
    int id;
    int x;
    int y;
    int w;
    int p;
    int tool_type;
};

/*
 * struct goodix_ts_event - touch event struct
 * @event_type: touch event type, touch data or
 *  request event
 * @event_data: event data
 */
struct goodix_ts_event {
    enum ts_event_type event_type;
    uint8_t fp_flag;     /* finger print DOWN flag */
    uint8_t request_code; /* represent the request type */
    uint8_t gesture_type;
    uint8_t gesture_data[GT9895_GESTURE_DATA_LEN];
    struct goodix_touch_data touch_data;
    struct goodix_pen_data pen_data;
};

struct goodix_fw_version {
    uint8_t rom_pid[6];               /* rom PID */
    uint8_t rom_vid[3];               /* Mask VID */
    uint8_t rom_vid_reserved;
    uint8_t patch_pid[8];             /* Patch PID */
    uint8_t patch_vid[4];             /* Patch VID */
    uint8_t patch_vid_reserved;
    uint8_t sensor_id;
    uint8_t reserved[2];
    uint16_t checksum;
};

struct goodix_ic_info_version {
    uint8_t info_customer_id;
    uint8_t info_version_id;
    uint8_t ic_die_id;
    uint8_t ic_version_id;
    uint32_t config_id;
    uint8_t config_version;
    uint8_t frame_data_customer_id;
    uint8_t frame_data_version_id;
    uint8_t touch_data_customer_id;
    uint8_t touch_data_version_id;
    uint8_t reserved[3];
};

struct goodix_ic_info_feature { /* feature info*/
    uint16_t freqhop_feature;
    uint16_t calibration_feature;
    uint16_t gesture_feature;
    uint16_t side_touch_feature;
    uint16_t stylus_feature;
};

struct goodix_ic_info_param { /* param */
    uint8_t drv_num;
    uint8_t sen_num;
    uint8_t button_num;
    uint8_t force_num;
    uint8_t active_scan_rate_num;
    uint16_t active_scan_rate[GT9895_MAX_SCAN_RATE_NUM];
    uint8_t mutual_freq_num;
    uint16_t mutual_freq[GT9895_MAX_SCAN_FREQ_NUM];
    uint8_t self_tx_freq_num;
    uint16_t self_tx_freq[GT9895_MAX_SCAN_FREQ_NUM];
    uint8_t self_rx_freq_num;
    uint16_t self_rx_freq[GT9895_MAX_SCAN_FREQ_NUM];
    uint8_t stylus_freq_num;
    uint16_t stylus_freq[GT9895_MAX_FREQ_NUM_STYLUS];
};

struct goodix_ic_info_misc { /* other data */
    uint32_t cmd_addr;
    uint16_t cmd_max_len;
    uint32_t cmd_reply_addr;
    uint16_t cmd_reply_len;
    uint32_t fw_state_addr;
    uint16_t fw_state_len;
    uint32_t fw_buffer_addr;
    uint16_t fw_buffer_max_len;
    uint32_t frame_data_addr;
    uint16_t frame_data_head_len;
    uint16_t fw_attr_len;
    uint16_t fw_log_len;
    uint8_t pack_max_num;
    uint8_t pack_compress_version;
    uint16_t stylus_struct_len;
    uint16_t mutual_struct_len;
    uint16_t self_struct_len;
    uint16_t noise_struct_len;
    uint32_t touch_data_addr;
    uint16_t touch_data_head_len;
    uint16_t point_struct_len;
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t mutual_rawdata_addr;
    uint32_t mutual_diffdata_addr;
    uint32_t mutual_refdata_addr;
    uint32_t self_rawdata_addr;
    uint32_t self_diffdata_addr;
    uint32_t self_refdata_addr;
    uint32_t iq_rawdata_addr;
    uint32_t iq_refdata_addr;
    uint32_t im_rawdata_addr;
    uint16_t im_readata_len;
    uint32_t noise_rawdata_addr;
    uint16_t noise_rawdata_len;
    uint32_t stylus_rawdata_addr;
    uint16_t stylus_rawdata_len;
    uint32_t noise_data_addr;
    uint32_t esd_addr;
};

struct goodix_ic_info {
    uint16_t length;
    struct goodix_ic_info_version version;
    struct goodix_ic_info_feature feature;
    struct goodix_ic_info_param parm;
    struct goodix_ic_info_misc misc;
};


class TouchDrvGT9895 :
    public TouchDrvInterface,
    public SensorCommon<TouchDrvGT9895>
{
    friend class SensorCommon<TouchDrvGT9895>;
public:
    TouchDrvGT9895();
    ~TouchDrvGT9895();

#if defined(ARDUINO)

    bool begin(PLATFORM_WIRE_TYPE &w, uint8_t addr = GT9895_SLAVE_ADDRESS_L, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL);

#elif defined(ESP_PLATFORM) && !defined(ARDUINO)
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr);
#else
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl);
#endif //ESP_IDF_VERSION
#endif

    bool begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback);
    void deinit();
    void reset();
    void sleep();
    void wakeup();
    void idle();

    uint8_t getSupportTouchPoint();
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1);
    bool isPressed();

    uint32_t getChipID();
    bool getResolution(int16_t *x, int16_t *y);

    const char *getModelName();
    void  setGpioCallback(gpio_mode_fptr_t mode_cb, gpio_write_fptr_t write_cb, gpio_read_fptr_t read_cb);

private:

    int is_risk_data(const uint8_t *data, int size);
    int checksum_cmp(const uint8_t *data, int size, int mode);

    int readVersion(struct goodix_fw_version *version);
    int convertChipInfo(struct goodix_ic_info *info, const uint8_t *data);
    void printChipInfo(struct goodix_ic_info *ic_info);
    int readChipInfo(struct goodix_ic_info *ic_info);

    void clearStatus();
    int getTouchData( uint8_t *pre_buf, uint32_t pre_buf_len);

    bool initImpl();

    int getReadMaskImpl();

protected:
    struct goodix_ts_event      __ts_event;
    struct goodix_fw_version    __version;
    struct goodix_ic_info       __ic_info;
};


