//
// BitBank Capacitive Touch Sensor Library
// written by Larry Bank
//
// Copyright 2023 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================

//
// Written for the many variants of ESP32 + Capacitive touch LCDs on the market
//
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#else
#define INPUT 0
#define OUTPUT 1
#include "driver/gpio.h"
#include "driver/i2c.h"
#endif // ARDUINO

#ifndef __BB_CAPTOUCH__
#define __BB_CAPTOUCH__

#define CT_SUCCESS 0
#define CT_ERROR -1
//
// Pre-configured device names
// Don't change the order of this list!
// the structure values follow it. Always
// add new values to the end
//
enum {
  CONFIG_T_QT_C6=0,
  CONFIG_CYD_550,
  CONFIG_T_DISPLAY_S3_PRO,
  CONFIG_T_DISPLAY_S3_LONG,
  CONFIG_CYD_22C,
  CONFIG_CYD_24C,
  CONFIG_CYD_128,
  CONFIG_CYD_35C,
  CONFIG_CYD_518,
  CONFIG_CYD_543,
  CONFIG_M5_CORE2,
  CONFIG_M5_CORES3,
  CONFIG_M5_PAPER,
  CONFIG_WT32_SC01_PLUS,
  CONFIG_MAKERFABS_480x480,
  CONFIG_MAKERFABS_320x480,
  CONFIG_T_DISPLAY_S3_AMOLED,
  CONFIG_COUNT
};
// structure holding the configurations
typedef struct bbct_config_tag {
  int8_t i8SDA, i8SCL, i8IRQ, i8RST;
} BBCT_CONFIG;

enum {
  CT_TYPE_UNKNOWN = 0,
  CT_TYPE_FT6X36,
  CT_TYPE_GT911,
  CT_TYPE_CST820,
  CT_TYPE_CST226,
  CT_TYPE_MXT144,
  CT_TYPE_AXS15231,
  CT_TYPE_COUNT
};

#define GT911_ADDR1 0x5D
#define GT911_ADDR2 0x14
#define FT6X36_ADDR 0x38
#define CST820_ADDR 0x15
#define CST226_ADDR 0x5A
#define MXT144_ADDR 0x4A
#define AXS15231_ADDR 0x3B

// CST8xx gestures
enum {
  GESTURE_NONE = 0,
  GESTURE_SWIPE_UP,
  GESTURE_SWIPE_DOWN,
  GESTURE_SWIPE_LEFT,
  GESTURE_SWIPE_RIGHT,
  GESTURE_SINGLE_CLICK,
  GESTURE_DOUBLE_CLICK = 0x0B,
  GESTURE_LONG_PRESS = 0x0C
};

typedef struct mxt_data_tag {
    uint16_t t2_encryption_status_address;
    uint16_t t5_message_processor_address;
    uint16_t t5_max_message_size;
    uint16_t t6_command_processor_address;
    uint16_t t7_powerconfig_address;
    uint16_t t8_acquisitionconfig_address;
    uint16_t t44_message_count_address;
    uint16_t t46_cte_config_address;
    uint16_t t100_multiple_touch_touchscreen_address;
    uint16_t t100_first_report_id;
} MXTDATA;

typedef struct mxt_object_tag {
    uint8_t type;
    uint16_t position;
    uint8_t size_minus_one;
    uint8_t instances_minus_one;
    uint8_t report_ids_per_instance;
} MXTOBJECT;

#define MXT_MESSAGE_SIZE 6

// CST820 registers
#define CST820_TOUCH_REGS 1

// GT911 registers
#define GT911_POINT_INFO 0x814E
#define GT911_POINT_1    0x814F
#define GT911_CONFIG_FRESH 0x8100
#define GT911_CONFIG_SIZE 0xb9
#define GT911_CONFIG_START 0x8047

// FT6x36 registers
#define TOUCH_REG_STATUS 0x02
#define TOUCH_REG_XH 0x03
#define TOUCH_REG_XL 0x04
#define TOUCH_REG_YH 0x05
#define TOUCH_REG_YL 0x06
#define TOUCH_REG_WEIGHT 0x07
#define TOUCH_REG_AREA 0x08
// register offset to info for the second touch point
#define PT2_OFFSET 6

#ifndef __TOUCHINFO_STRUCT__
#define __TOUCHINFO_STRUCT__

typedef struct _fttouchinfo
{
  int count;
  uint16_t x[5], y[5];
  uint8_t pressure[5], area[5];
} TOUCHINFO;
#endif

//extern TwoWire* myWire;

class BBCapTouch
{
public:
    BBCapTouch() { _iOrientation = 0;}
//    ~BBCapTouch() { Wire.end(); }
    ~BBCapTouch() { myWire->end(); }

#ifdef ARDUINO
    int init(int iConfigName);
    int init(int iSDA, int iSCL, int iRST=-1, int iINT=-1, uint32_t u32Speed=400000, TwoWire* _myWire=&Wire);
#else
    int init(int iSDA, int iSCL, int iRST=-1, int iINT=-1, uint32_t u32Speed=400000);
#endif
    int getSamples(TOUCHINFO *pTI);
    int sensorType(void);
    int setOrientation(int iOrientation, int iWidth, int iHeight);

protected:
    void reset(int iResetPin);
 
private:
    int _iAddr;
    int _iType;
    int _iOrientation, _iWidth, _iHeight;
    MXTDATA _mxtdata;
#ifdef ARDUINO
    TwoWire* myWire;
#else
    void pinMode(uint8_t u8Pin, uint8_t u8Mode);
    void digitalWrite(uint8_t u8Pin, uint8_t u8State);
#endif
    int initMXT(void);
    void fixSamples(TOUCHINFO *pTI);
    bool I2CTest(uint8_t u8Addr);
    int I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen);
    int I2CReadRegister(uint8_t u8Addr, uint8_t u8Register, uint8_t *pData, int iLen);
    int I2CReadRegister16(uint8_t u8Addr, uint16_t u16Register, uint8_t *pData, int iLen);
    int I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen);
}; // class BBCapTouch
#endif // __BB_CAPTOUCH__
