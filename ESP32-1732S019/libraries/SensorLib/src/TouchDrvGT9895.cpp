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
 * @file      TouchDrvGT9895.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#include "TouchDrvGT9895.hpp"

TouchDrvGT9895::TouchDrvGT9895()
{
#if defined(ARDUINO)
    __wire = &Wire;
    __sda = DEFAULT_SDA;
    __scl = DEFAULT_SCL;
#endif
    __rst = SENSOR_PIN_NONE;
    __irq = SENSOR_PIN_NONE;
    __addr = GT9895_SLAVE_ADDRESS_L;
}

TouchDrvGT9895::~TouchDrvGT9895()
{
    deinit();
}

#if defined(ARDUINO)
bool TouchDrvGT9895::begin(PLATFORM_WIRE_TYPE &w, uint8_t addr, int sda, int scl )
{
    return SensorCommon::begin(w, addr, sda, scl);
}

#elif defined(ESP_PLATFORM) && !defined(ARDUINO)

#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
bool TouchDrvGT9895::begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr)
{
    return SensorCommon::begin(i2c_dev_bus_handle, addr);
}
#else
bool TouchDrvGT9895::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    return SensorCommon::begin(port_num, addr, sda, scl);
}
#endif //ESP_IDF_VERSION

#endif

bool TouchDrvGT9895::begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
{
    return SensorCommon::begin(addr, readRegCallback, writeRegCallback);
}


void TouchDrvGT9895::deinit()
{
    // end();
}


void TouchDrvGT9895::reset()
{
    if (__rst != SENSOR_PIN_NONE) {
        this->setGpioMode(__rst, OUTPUT);
        this->setGpioLevel(__rst, HIGH);
        delay(10);
        this->setGpioLevel(__rst, LOW);
        delay(30);
        this->setGpioLevel(__rst, HIGH);
        delay(100);
    }
    if (__irq != SENSOR_PIN_NONE) {
        this->setGpioMode(__irq, INPUT);
    }
}

void TouchDrvGT9895::sleep()
{
    if (__irq != SENSOR_PIN_NONE) {
        this->setGpioMode(__irq, OUTPUT);
        this->setGpioLevel(__irq, LOW);
    }

    uint8_t sleep_cmd[] = {
        ((GT9895_REG_CMD >> 24) & 0xFF),
        ((GT9895_REG_CMD >> 16) & 0xFF),
        ((GT9895_REG_CMD >> 8) & 0xFF),
        (GT9895_REG_CMD & 0xFF),
        0x00, 0x00, 0x04, 0x84, 0x88, 0x00
    };
    writeBuffer(sleep_cmd, sizeof(sleep_cmd));
}

void TouchDrvGT9895::wakeup()
{
    if (__irq != SENSOR_PIN_NONE) {
        this->setGpioMode(__irq, OUTPUT);
        this->setGpioLevel(__irq, HIGH);
        delay(8);
    }
    reset();
}

void TouchDrvGT9895::idle()
{

}

uint8_t TouchDrvGT9895::getSupportTouchPoint()
{
    return GT9895_MAX_TOUCH;
}

uint8_t TouchDrvGT9895::getPoint(int16_t *x_array, int16_t *y_array, uint8_t size)
{
    uint8_t buffer[32] = {0};
    uint8_t event_status;

    int length = GT9895_IRQ_EVENT_HEAD_LEN + GT9895_BYTES_PER_POINT * 2 + GT9895_COOR_DATA_CHECKSUM_SIZE;

    if (writeThenRead(GT9895_REG_POINT, buffer, length) == DEV_WIRE_ERR) {
        return 0;
    }

    if (buffer[0] == 0x00) {
        return 0;
    }

    if (checksum_cmp(buffer, GT9895_IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
        // log_e("touch head checksum err[%*ph]", GT9895_IRQ_EVENT_HEAD_LEN, buffer);
        return 0;
    }

    event_status = buffer[0];

    if (event_status & GT9895_TOUCH_EVENT) {

        int  touchNum = getTouchData(buffer, length);

        if (!touchNum) {
            return 0;
        }

        if ( x_array && y_array && size) {
            uint8_t length = size < touchNum ? size : touchNum;
            for (int i = 0; i < length; ++i) {
                x_array[i] = __ts_event.touch_data.coords[i].x;
                y_array[i] = __ts_event.touch_data.coords[i].y;
            }
            updateXY(touchNum, x_array, y_array);
        }

        return touchNum;
    }

#if 0
    if (event_status & GT9895_REQUEST_EVENT) {
        __ts_event.event_type = EVENT_REQUEST;

        if (buffer[2] == BRL_REQUEST_CODE_CONFIG)
            __ts_event.request_code = REQUEST_TYPE_CONFIG;
        else if (buffer[2] == BRL_REQUEST_CODE_RESET)
            __ts_event.request_code = REQUEST_TYPE_RESET;
        else
            log_e("unsupported request code 0x%x", buffer[2]);
    }

    if (event_status & GT9895_GESTURE_EVENT) {
        __ts_event.event_type = EVENT_GESTURE;
        __ts_event.gesture_type = buffer[4];
        memcpy(__ts_event.gesture_data, &buffer[8],  GT9895_GESTURE_DATA_LEN);
    }
#endif

    clearStatus();

    return 0;
}

bool TouchDrvGT9895::isPressed()
{
    if (__irq != SENSOR_PIN_NONE) {
        return this->getGpioLevel(__irq) == LOW;
    } else {
        return getPoint(NULL, NULL, 0);
    }
    return false;
}

uint32_t TouchDrvGT9895::getChipID()
{
    return (uint32_t)strtol((const char *)__version.patch_pid, NULL, 16);
}


bool TouchDrvGT9895::getResolution(int16_t *x, int16_t *y)
{
    return 0;
}

const char *TouchDrvGT9895::getModelName()
{
    return "GT9895";
}

void  TouchDrvGT9895::setGpioCallback(gpio_mode_fptr_t mode_cb,
                                      gpio_write_fptr_t write_cb,
                                      gpio_read_fptr_t read_cb)
{
    SensorCommon::setGpioModeCallback(mode_cb);
    SensorCommon::setGpioWriteCallback(write_cb);
    SensorCommon::setGpioReadCallback(read_cb);
}

int TouchDrvGT9895::is_risk_data(const uint8_t *data, int size)
{
    int zero_count =  0;
    int ff_count = 0;

    for (int i = 0; i < size; i++) {
        if (data[i] == 0)
            zero_count++;
        else if (data[i] == 0xFF)
            ff_count++;
    }
    if (zero_count == size || ff_count == size) {
        log_e("warning data is all %s\n", zero_count == size ? "0x00" : "0xFF");
        return 1;
    }
    return 0;
}

int TouchDrvGT9895::checksum_cmp(const uint8_t *data, int size, int mode)
{
    uint32_t cal_checksum = 0;
    uint32_t r_checksum = 0;
    if (mode == CHECKSUM_MODE_U8_LE) {
        if (size < 2)
            return 1;
        for (int i = 0; i < size - 2; i++)
            cal_checksum += data[i];
        r_checksum = data[size - 2] + (data[size - 1] << 8);
        return (cal_checksum & 0xFFFF) == r_checksum ? 0 : 1;
    }

    if (size < 4)
        return 1;
    for (int i = 0; i < size - 4; i += 2)
        cal_checksum += data[i] + (data[i + 1] << 8);
    r_checksum = data[size - 4] + (data[size - 3] << 8) +
                 (data[size - 2] << 16) + (data[size - 1] << 24);
    return cal_checksum == r_checksum ? 0 : 1;
}

int TouchDrvGT9895::readVersion(struct goodix_fw_version *version)
{
    int ret = 0;
    uint8_t buffer[sizeof(struct goodix_fw_version)] = {0};
    uint8_t temp_pid[8] = {0};

    if (!version) {
        return DEV_WIRE_ERR;
    }

    for (int i = 0; i < 2; i++) {
        if (writeThenRead(GT9895_REG_FW_VERSION, buffer, sizeof(buffer)) == DEV_WIRE_ERR) {
            log_e("read fw version: %d, retry %d", ret, i);
            ret = DEV_WIRE_ERR;
            delay(5);
            continue;
        }
        if (!checksum_cmp(buffer, sizeof(buffer), CHECKSUM_MODE_U8_LE)) {
            ret = DEV_WIRE_NONE;
            break;
        }

        log_e("Invalid fw version: checksum error!");
        log_e("Firmware version:%*ph", (int)sizeof(buffer), buffer);
        ret = DEV_WIRE_ERR;
        delay(15);

    }
    if (ret == DEV_WIRE_ERR) {
        log_e("Failed get valid firmware version");
        return ret;
    }

    memcpy(version, buffer, sizeof(*version));
    memcpy(temp_pid, version->rom_pid, sizeof(version->rom_pid));
    log_d("Rom_pid:%s", (const char *)temp_pid);
    log_d("Rom_vid:%*p", (int)sizeof(version->rom_vid), version->rom_vid);
    log_d("PID:%s", (const char *)version->patch_pid);
    log_d("VID:%*p", (int)sizeof(version->patch_vid), version->patch_vid);
    log_d("Sensor ID:%d", version->sensor_id);

    return DEV_WIRE_NONE;
}

int TouchDrvGT9895::convertChipInfo(struct goodix_ic_info *info, const uint8_t *data)
{
    int i = 0;
    struct goodix_ic_info_version *version = &info->version;
    struct goodix_ic_info_feature *feature = &info->feature;
    struct goodix_ic_info_param *parm = &info->parm;
    struct goodix_ic_info_misc *misc = &info->misc;

    info->length = *((uint16_t *)data);
    data += 2;

    memcpy(version, data, sizeof(*version));
    data += sizeof(struct goodix_ic_info_version);

    memcpy(feature, data, sizeof(*feature));
    data += sizeof(struct goodix_ic_info_feature);

    parm->drv_num = *(data++);
    parm->sen_num = *(data++);
    parm->button_num = *(data++);
    parm->force_num = *(data++);
    parm->active_scan_rate_num = *(data++);

    if (parm->active_scan_rate_num > GT9895_MAX_SCAN_RATE_NUM) {
        log_e("Invalid scan rate num %d > %d", parm->active_scan_rate_num, GT9895_MAX_SCAN_RATE_NUM);
        return DEV_WIRE_ERR;
    }
    for (i = 0; i < parm->active_scan_rate_num; i++)
        parm->active_scan_rate[i] = *((uint16_t *)(data + i * 2));

    data += parm->active_scan_rate_num * 2;
    parm->mutual_freq_num = *(data++);
    if (parm->mutual_freq_num > GT9895_MAX_SCAN_FREQ_NUM) {
        log_e("invalid mutual freq num %d > %d", parm->mutual_freq_num, GT9895_MAX_SCAN_FREQ_NUM);
        return DEV_WIRE_ERR;
    }
    for (i = 0; i < parm->mutual_freq_num; i++)
        parm->mutual_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->mutual_freq_num * 2;
    parm->self_tx_freq_num = *(data++);
    if (parm->self_tx_freq_num > GT9895_MAX_SCAN_FREQ_NUM) {
        log_e("Invalid tx freq num %d > %d", parm->self_tx_freq_num, GT9895_MAX_SCAN_FREQ_NUM);
        return DEV_WIRE_ERR;
    }
    for (i = 0; i < parm->self_tx_freq_num; i++)
        parm->self_tx_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->self_tx_freq_num * 2;
    parm->self_rx_freq_num = *(data++);
    if (parm->self_rx_freq_num > GT9895_MAX_SCAN_FREQ_NUM) {
        log_e("Invalid rx freq num %d > %d",  parm->self_rx_freq_num, GT9895_MAX_SCAN_FREQ_NUM);
        return DEV_WIRE_ERR;
    }
    for (i = 0; i < parm->self_rx_freq_num; i++)
        parm->self_rx_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->self_rx_freq_num * 2;
    parm->stylus_freq_num = *(data++);
    if (parm->stylus_freq_num > GT9895_MAX_FREQ_NUM_STYLUS) {
        log_e("Invalid stylus freq num %d > %d", parm->stylus_freq_num, GT9895_MAX_FREQ_NUM_STYLUS);
        return DEV_WIRE_ERR;
    }
    for (i = 0; i < parm->stylus_freq_num; i++)
        parm->stylus_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->stylus_freq_num * 2;
    memcpy(misc, data, sizeof(*misc));
    return DEV_WIRE_NONE;
}

void TouchDrvGT9895::printChipInfo(struct goodix_ic_info *ic_info)
{
    struct goodix_ic_info_version *version = &ic_info->version;
    struct goodix_ic_info_feature *feature = &ic_info->feature;
    struct goodix_ic_info_param *parm = &ic_info->parm;
    struct goodix_ic_info_misc *misc = &ic_info->misc;

    (void)version;
    (void)feature;
    (void)parm;
    (void)misc;

    log_d("ic_info_length:                %d", ic_info->length);
    log_d("info_customer_id:              0x%01X", version->info_customer_id);
    log_d("info_version_id:               0x%01X", version->info_version_id);
    log_d("ic_die_id:                     0x%01X", version->ic_die_id);
    log_d("ic_version_id:                 0x%01X", version->ic_version_id);
    log_d("config_id:                     0x%4lX", version->config_id);
    log_d("config_version:                0x%01X", version->config_version);
    log_d("frame_data_customer_id:        0x%01X", version->frame_data_customer_id);
    log_d("frame_data_version_id:         0x%01X", version->frame_data_version_id);
    log_d("touch_data_customer_id:        0x%01X", version->touch_data_customer_id);
    log_d("touch_data_version_id:         0x%01X", version->touch_data_version_id);
    log_d("freq_hop_feature:              0x%04X", feature->freqhop_feature);
    log_d("calibration_feature:           0x%04X", feature->calibration_feature);
    log_d("gesture_feature:               0x%04X", feature->gesture_feature);
    log_d("side_touch_feature:            0x%04X", feature->side_touch_feature);
    log_d("stylus_feature:                0x%04X", feature->stylus_feature);
    log_d("Drv*Sen,Button,Force num:      %u x %u, %u, %u", parm->drv_num, parm->sen_num, parm->button_num, parm->force_num);
    log_d("Cmd:                           0x%04lX, %u", misc->cmd_addr, misc->cmd_max_len);
    log_d("Cmd-Reply:                     0x%04lX, %u", misc->cmd_reply_addr, misc->cmd_reply_len);
    log_d("FW-State:                      0x%04lX, %u", misc->fw_state_addr, misc->fw_state_len);
    log_d("FW-Buffer:                     0x%04lX, %u", misc->fw_buffer_addr, misc->fw_buffer_max_len);
    log_d("Touch-Data:                    0x%04lX, %u", misc->touch_data_addr, misc->touch_data_head_len);
    log_d("point_struct_len:              %u", misc->point_struct_len);
    log_d("mutual_raw_data_addr:          0x%04lX", misc->mutual_rawdata_addr);
    log_d("mutual_diff_data_addr:         0x%04lX", misc->mutual_diffdata_addr);
    log_d("self_raw_data_addr:            0x%04lX", misc->self_rawdata_addr);
    log_d("self_diff_data_addr:           0x%04lX", misc->self_diffdata_addr);
    log_d("stylus_raw_data_addr:          0x%04lX, %u", misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
    log_d("esd_addr:                      0x%04lX", misc->esd_addr);
}

int TouchDrvGT9895::readChipInfo(struct goodix_ic_info *ic_info)
{
    int  i = 0;
    uint16_t length = 0;
    uint8_t afe_data[GT9895_INFO_MAX_LENGTH] = {0};

    for (i = 0; i < 3; i++) {
        if (writeThenRead(GT9895_REG_INFO, (uint8_t *)&length, sizeof(length)) == DEV_WIRE_ERR) {
            log_e("Failed get ic info length");
            delay(5);
            continue;
        }
        if (length >= GT9895_INFO_MAX_LENGTH || length == 0) {
            log_e("Invalid ic info length %d, retry %d", length, i);
            continue;
        }
        if (writeThenRead(GT9895_REG_INFO, afe_data, length) == DEV_WIRE_ERR) {
            log_e("Failed get ic info data");
            delay(5);
            continue;
        }
        /* judge whether the data is valid */
        if (is_risk_data((const uint8_t *)afe_data, length)) {
            log_e("Firmware info data invalid");
            delay(5);
            continue;
        }
        if (checksum_cmp((const uint8_t *)afe_data, length, CHECKSUM_MODE_U8_LE)) {
            log_e("Firmware info checksum error!");
            delay(5);
            continue;
        }
        break;
    }
    if (i == 3) {
        log_e("Failed get ic info");
        return DEV_WIRE_ERR;
    }
    if (convertChipInfo(ic_info, afe_data) == DEV_WIRE_ERR) {
        log_e("Convert ic info encounter error");
        return DEV_WIRE_ERR;
    }
    printChipInfo(ic_info);
    /* check some key info */
    if (!ic_info->misc.cmd_addr || !ic_info->misc.fw_buffer_addr ||
            !ic_info->misc.touch_data_addr) {
        log_e("cmd_addr fw_buf_addr and touch_data_addr is null");
        return DEV_WIRE_ERR;
    }
    return DEV_WIRE_NONE;
}

void TouchDrvGT9895::clearStatus()
{
    uint8_t buffer[5] =  { 0x00, 0x01, 0x03, 0x08, 0x00};
    writeBuffer(buffer, 5);
}

int TouchDrvGT9895::getTouchData(uint8_t *pre_buf, uint32_t pre_buf_len)
{
    uint8_t touch_num = 0;
    uint8_t point_type = 0;
    uint8_t buffer[GT9895_IRQ_EVENT_HEAD_LEN + GT9895_BYTES_PER_POINT * GT9895_MAX_TOUCH + 2];

    /* clean event buffer */
    memset(&__ts_event, 0, sizeof(__ts_event));
    /* copy pre-data to buffer */
    memcpy(buffer, pre_buf, pre_buf_len);

    touch_num = buffer[2] & 0x0F;
    if (touch_num > GT9895_MAX_TOUCH) {
        log_e("invalid touch num %d", touch_num);
        return 0;
    }

    if (touch_num > 2) {
        uint32_t addr = GT9895_REG_POINT + pre_buf_len;
        if (writeThenRead(addr, &buffer[pre_buf_len], (touch_num - 2) * GT9895_BYTES_PER_POINT) == DEV_WIRE_ERR) {
            log_e("Failed get touch data");
            return 0;
        }
    }

    if (touch_num > 0) {
        point_type = buffer[GT9895_IRQ_EVENT_HEAD_LEN] & 0x0F;
        if (point_type == GT9895_POINT_TYPE_STYLUS || point_type == GT9895_POINT_TYPE_STYLUS_HOVER) {
            if (checksum_cmp(&buffer[GT9895_IRQ_EVENT_HEAD_LEN], GT9895_BYTES_PER_POINT * 2 + 2, CHECKSUM_MODE_U8_LE)) {
                // log_e("Touch data checksum error");
                return 0;
            }
        } else {
            if (checksum_cmp(&buffer[GT9895_IRQ_EVENT_HEAD_LEN], touch_num * GT9895_BYTES_PER_POINT + 2, CHECKSUM_MODE_U8_LE)) {
                // log_e("Touch data checksum error");
                return 0;
            }
        }
    }

    __ts_event.fp_flag = pre_buf[0] & GT9895_FP_EVENT;
    /* finger info */
    __ts_event.event_type = EVENT_TOUCH;

    uint32_t id = 0, x = 0, y = 0, w = 0;
    uint8_t *pdat = &buffer[GT9895_IRQ_EVENT_HEAD_LEN];
    for (int i = 0; i < touch_num; i++) {
        id = (pdat[0] >> 4) & 0x0F;
        if (id >= GT9895_MAX_TOUCH) {
            log_e("Invalid finger id");
            __ts_event.touch_data.touch_num = 0;
            return 0;
        }
        x = *((uint16_t *)(pdat + 2));
        y = *((uint16_t *)(pdat + 4));
        w = *((uint16_t *)(pdat + 6));
        __ts_event.touch_data.coords[id].status = TS_TOUCH;
        __ts_event.touch_data.coords[id].x = x;
        __ts_event.touch_data.coords[id].y = y;
        __ts_event.touch_data.coords[id].w = w;
        pdat += GT9895_BYTES_PER_POINT;
    }

    __ts_event.touch_data.touch_num = touch_num;
    return touch_num;
}

bool TouchDrvGT9895::initImpl()
{
    if (__irq != SENSOR_PIN_NONE) {
        this->setGpioMode(__irq, INPUT);
    }

    reset();

    if (readVersion(&__version) != DEV_WIRE_NONE) {
        return false;
    }

    readChipInfo(&__ic_info);

    return true;
}

int TouchDrvGT9895::getReadMaskImpl()
{
    return 0X80;
}

/*
[  7142][I][SensorCommon.tpp:65] begin(): Using Arduino Wire interface.
[  7148][W][Wire.cpp:301] begin(): Bus already started in Master Mode.
[  7197][D][TouchDrvGT9895.cpp:348] readVersion(): Rom_pid:BERLIN
[  7203][D][TouchDrvGT9895.cpp:349] readVersion(): Rom_vid:0x3fc95c33
[  7210][D][TouchDrvGT9895.cpp:350] readVersion(): PID:9895
[  7215][D][TouchDrvGT9895.cpp:351] readVersion(): VID:0x3fc95c3f
[  7221][D][TouchDrvGT9895.cpp:352] readVersion(): Sensor ID:255
[  7244][I][TouchDrvGT9895.cpp:435] printChipInfo(): ic_info_length:                173
[  7252][I][TouchDrvGT9895.cpp:436] printChipInfo(): info_customer_id:              0x1
[  7260][I][TouchDrvGT9895.cpp:437] printChipInfo(): info_version_id:               0x0
[  7267][I][TouchDrvGT9895.cpp:438] printChipInfo(): ic_die_id:                     0x0
[  7275][I][TouchDrvGT9895.cpp:439] printChipInfo(): ic_version_id:                 0x0
[  7283][I][TouchDrvGT9895.cpp:440] printChipInfo(): config_id:                     0x650BFC22
[  7291][I][TouchDrvGT9895.cpp:441] printChipInfo(): config_version:                0x2
[  7299][I][TouchDrvGT9895.cpp:442] printChipInfo(): frame_data_customer_id:        0x1
[  7307][I][TouchDrvGT9895.cpp:443] printChipInfo(): frame_data_version_id:         0x0
[  7315][I][TouchDrvGT9895.cpp:444] printChipInfo(): touch_data_customer_id:        0x1
[  7323][I][TouchDrvGT9895.cpp:445] printChipInfo(): touch_data_version_id:         0x0
[  7330][I][TouchDrvGT9895.cpp:446] printChipInfo(): freq_hop_feature:              0x0000
[  7338][I][TouchDrvGT9895.cpp:447] printChipInfo(): calibration_feature:           0x0000
[  7346][I][TouchDrvGT9895.cpp:448] printChipInfo(): gesture_feature:               0x0000
[  7354][I][TouchDrvGT9895.cpp:449] printChipInfo(): side_touch_feature:            0x0000
[  7363][I][TouchDrvGT9895.cpp:450] printChipInfo(): stylus_feature:                0x0000
[  7371][I][TouchDrvGT9895.cpp:452] printChipInfo(): Drv*Sen,Button,Force num:      10 x 23, 0, 0
[  7379][I][TouchDrvGT9895.cpp:453] printChipInfo(): Cmd:                           0x10174, 16
[  7388][I][TouchDrvGT9895.cpp:454] printChipInfo(): Cmd-Reply:                     0x10184, 16
[  7396][I][TouchDrvGT9895.cpp:455] printChipInfo(): FW-State:                      0x10218, 92
[  7405][I][TouchDrvGT9895.cpp:456] printChipInfo(): FW-Buffer:                     0x13D80, 4096
[  7413][I][TouchDrvGT9895.cpp:457] printChipInfo(): Touch-Data:                    0x10308, 8
[  7422][I][TouchDrvGT9895.cpp:458] printChipInfo(): point_struct_len:              8
[  7429][I][TouchDrvGT9895.cpp:459] printChipInfo(): mutual_raw_data_addr:           0x13830
[  7438][I][TouchDrvGT9895.cpp:460] printChipInfo(): mutual_diff_data_addr:          0x11224
[  7446][I][TouchDrvGT9895.cpp:461] printChipInfo(): self_raw_data_addr:             0x137C4
[  7454][I][TouchDrvGT9895.cpp:462] printChipInfo(): self_diff_data_addr:            0x13758
[  7462][I][TouchDrvGT9895.cpp:463] printChipInfo(): stylus_raw_data_addr:           0x0000, 0
[  7471][I][TouchDrvGT9895.cpp:464] printChipInfo(): esd_addr:                      0x10170
*/













