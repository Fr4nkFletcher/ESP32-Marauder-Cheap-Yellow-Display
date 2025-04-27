//
// BitBank Capactive Touch Sensor Library
// Written by Larry Bank
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
#include "bb_captouch.h"

static const BBCT_CONFIG _configs[] = {
       // sda, scl, irq, rst
       {21, 22, 7, 23}, // CONFIG_T_QT_C6
       {19, 20, -1, 38}, // CONFIG_CYD_550
       {5, 6, 7, 13}, // T-Display S3 Pro
       {15, 20, 11, 16}, // T-Display-S3-Long
       {21, 22, -1, -1}, // CYD_22C
       {33, 32, 21, 25}, // CYD_24C
       {4, 5, 0, 1}, // CYD_128
       {33, 32, 21, 25}, // CYD_35C
       {7, 8, 41, 40}, // CYD_518
       {8, 4, 3, -1}, // CYD_543
       {21, 22, 39, -1}, // M5_CORE2
       {12, 11, -1, -1}, // M5_CORES3
       {21, 22, 36, -1}, // M5_PAPER
       {6, 5 ,7, -1}, // WT32_SC01_PLUS
       {17, 18, -1, 38}, // MakerFabs 4" 480x480
       {38,39,40,-1}, // MakerFabs 3.5" 320x480
       {7, 6, 9, 8}, // CONFIG_T_DISPLAY_S3_AMOLED (1.64")
       {0,0,0,0}
    };

#ifndef ARDUINO
void BBCapTouch::pinMode(uint8_t u8Pin, uint8_t u8Mode)
{
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1 << u8Pin);
    io_conf.pull_down_en = 0; //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    if (u8Mode == INPUT) {
        io_conf.mode = GPIO_MODE_INPUT;
    } else { // must be output
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    gpio_config(&io_conf); //configure GPIO with the given settings
} /* pinMode() */
void BBCapTouch::digitalWrite(uint8_t u8Pin, uint8_t u8State)
{
    gpio_set_level(u8Pin, u8State);
} /* digitalWrite() */
#endif // !ARDUINO
void BBCapTouch::reset(int iRST)
{
     pinMode(iRST, OUTPUT);
     digitalWrite(iRST, LOW);
     delay(100);
     digitalWrite(iRST, HIGH);
     delay(250);
}  /* reset() */

//
// Initalize the MXT144 - an overly complicated mess of a touch sensor
//
int BBCapTouch::initMXT(void)
{
uint8_t ucTemp[32];
int i, iObjCount, iReportID;
uint16_t u16, u16Offset;

// Read information block (first 7 bytes of address space)
   I2CReadRegister16(_iAddr, 0, ucTemp, 7);
   iObjCount = ucTemp[6];
   if (iObjCount < 1 || iObjCount >64) { // invalid number of items
      return CT_ERROR;
   }
   u16Offset = 7; // starting offset of first object
   // Read the objects one by one to get the memory offests to the info we will need
   iReportID = 1;
   Serial.printf("object count = %d\n", iObjCount);
   for (i=0; i<iObjCount; i++) {
      I2CReadRegister16(_iAddr, u16Offset, ucTemp, 6);
      Serial.printf("object %d, type %d\n", i, ucTemp[0]);
      u16 = ucTemp[1] | (ucTemp[2] << 8);
      switch (ucTemp[0]) {
         case 2:
            _mxtdata.t2_encryption_status_address = u16;
            break;
         case 5:
            _mxtdata.t5_message_processor_address = u16;
            _mxtdata.t5_max_message_size = ucTemp[4] - 1;
            break;
         case 6:
            _mxtdata.t6_command_processor_address = u16;
            break;
         case 7:
            _mxtdata.t7_powerconfig_address = u16;
            break;
         case 8:
            _mxtdata.t8_acquisitionconfig_address = u16;
            break;
         case 44:
            _mxtdata.t44_message_count_address = u16;
            break;
         case 46:
            _mxtdata.t46_cte_config_address = u16;
            break;
         case 100:
            _mxtdata.t100_multiple_touch_touchscreen_address = u16;
            _mxtdata.t100_first_report_id = iReportID;
            break;
         default:
            break;
      } // switch on type
      u16Offset += 6; // size in bytes of an object table
      iReportID += ucTemp[5] * (ucTemp[4] + 1);
   } // for each report
Serial.printf("init success, count offset = %d\n", _mxtdata.t44_message_count_address);
   return CT_SUCCESS;
} /* initMXT() */

//
// Initialize the library
// It only needs to initialize the I2C interface; the chip is ready
//
#ifdef ARDUINO
int BBCapTouch::init(int iSDA, int iSCL, int iRST, int iINT, uint32_t u32Speed, TwoWire* _myWire)
#else
int BBCapTouch::init(int iSDA, int iSCL, int iRST, int iINT, uint32_t u32Speed)
#endif
{
uint8_t ucTemp[4];

#ifdef ARDUINO
    myWire = _myWire;
    myWire->begin(iSDA, iSCL); // this is specific to ESP32 MCUs
    myWire->setClock(u32Speed);
    myWire->setTimeout(100);
#else
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = iSDA,
        .scl_io_num = iSCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = u32Speed,
    };
    i2c_driver_delete(I2C_NUM_0); // remove driver (if installed)
    i2c_param_config(I2C_NUM_0, &conf); // configure I2C device 0
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0); // configure with no send or receive buffers
#endif
    _iType = CT_TYPE_UNKNOWN;

    if (iRST != -1) {
       reset(iRST);
    }

#ifdef FUTURE
    if (I2CTest(AXS15231_ADDR)) {
       _iType = CT_TYPE_AXS15231;
       _iAddr = AXS15231_ADDR;
       if (iRST != -1) {
          reset(iRST);
       }
       return CT_SUCCESS;
    } // AXS15231
#endif

    if (I2CTest(MXT144_ADDR)) {
       _iType = CT_TYPE_MXT144;
       _iAddr = MXT144_ADDR;
       if (iINT != -1) {
          pinMode(iINT, INPUT);
       }
       if (iRST != -1) {
          reset(iRST);
       }
       return initMXT();
    }
    if (I2CTest(CST226_ADDR)) {
       _iType = CT_TYPE_CST226;
       _iAddr = CST226_ADDR;
       if (iINT != -1) {
          pinMode(iINT, INPUT);
       }
       return CT_SUCCESS;
    }
    if (I2CTest(GT911_ADDR1) || I2CTest(GT911_ADDR2)) {
       _iType = CT_TYPE_GT911;
    }
    if (_iType == CT_TYPE_GT911) { // reset the sensor to start it
       pinMode(iRST, OUTPUT);
       pinMode(iINT, OUTPUT);
       digitalWrite(iINT, LOW);
       digitalWrite(iRST, LOW);
       delay(5);
       digitalWrite(iINT, LOW); // set I2C addr to ADDR1
       delay(1);
       digitalWrite(iRST, HIGH); // when it comes out of reset, it samples INT
       delay(10);
       digitalWrite(iINT, LOW);
       delay(50);
       pinMode(iINT, INPUT);
       // double check the I2C addr in case it changed
       if (I2CTest(GT911_ADDR1)) {
          _iAddr = GT911_ADDR1;
       } else if (I2CTest(GT911_ADDR2)) {
          _iAddr = GT911_ADDR2;
       }
    } else if (I2CTest(FT6X36_ADDR)) {
       _iType = CT_TYPE_FT6X36;
       _iAddr = FT6X36_ADDR;
    } else if (I2CTest(CST820_ADDR)) {
       _iType = CT_TYPE_CST820;
       _iAddr = CST820_ADDR;
       if (iRST != -1) {
          reset(iRST);
       }
    } else {
#ifdef ARDUINO
       myWire->end();
#else
       i2c_driver_delete(I2C_NUM_0);
#endif
       return CT_ERROR; // no device found
    }
    return CT_SUCCESS;
} /* init() */

// Initialize the touch controller from a pre-defined configuration name
int BBCapTouch::init(int iConfigName)
{
   if (iConfigName < 0 || iConfigName >= CONFIG_COUNT) {
       return CT_ERROR;
   }
   return init(_configs[iConfigName].i8SDA,
               _configs[iConfigName].i8SCL,
               _configs[iConfigName].i8RST,
               _configs[iConfigName].i8IRQ,
#ifdef ARDUINO
               100000, &Wire);
#else
               100000);
#endif
} /* init() */
//
// Test if an I2C device is monitoring an address
// return true if it responds, false if no response
//
bool BBCapTouch::I2CTest(uint8_t u8Addr)
{
  // Check if a device acknowledges the address.
#ifdef ARDUINO
  myWire->beginTransmission(u8Addr);
  return(myWire->endTransmission(true) == 0);
#else // allow 100ms for device to respond
    uint8_t c = 0;
    return (i2c_master_write_to_device(I2C_NUM_0, u8Addr, &c, 1, 10) == ESP_OK);
#endif
} /* I2CTest() */
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int BBCapTouch::I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen)
{
  int rc = 0;

#ifdef ARDUINO
    myWire->beginTransmission(u8Addr);
    myWire->write(pData, (uint8_t)iLen);
    rc = !myWire->endTransmission();
#else
    rc = (i2c_master_write_to_device(I2C_NUM_0, u8Addr, pData, iLen, 100) == ESP_OK);
#endif
    return rc;
} /* I2CWrite() */
//
// Read N bytes starting at a specific 16-bit I2C register
//
int BBCapTouch::I2CReadRegister16(uint8_t u8Addr, uint16_t u16Register, uint8_t *pData, int iLen)
{
  int i = 0;

#ifdef ARDUINO
  myWire->beginTransmission(u8Addr);
  if (_iType == CT_TYPE_MXT144) { // little endian
    myWire->write((uint8_t)u16Register); // low byte
    myWire->write((uint8_t)(u16Register>>8)); // high byte
  } else { // big endian address
    myWire->write((uint8_t)(u16Register>>8)); // high byte
    myWire->write((uint8_t)u16Register); // low byte
  }
  myWire->endTransmission();
  myWire->requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
      pData[i++] = myWire->read();
  }
#else
    uint8_t ucTemp[4];
    int rc;
    if (_iType == CT_TYPE_MXT144) { // little endian
        ucTemp[1] = (uint8_t)(u16Register>>8); // high byte
        ucTemp[0] = (uint8_t)u16Register; // low byte
    } else {
        ucTemp[0] = (uint8_t)(u16Register>>8); // high byte
        ucTemp[1] = (uint8_t)u16Register; // low byte
    }
    i2c_master_write_read_device(I2C_NUM_0, u8Addr, ucTemp, 2, pData, iLen, 100);
    if (rc == ESP_OK) {
            i = iLen;
    }
#endif
  return i;

} /* I2CReadRegister16() */
//
// Read N bytes starting at a specific I2C internal register
// returns 1 for success, 0 for error
//
int BBCapTouch::I2CReadRegister(uint8_t u8Addr, uint8_t u8Register, uint8_t *pData, int iLen)
{
  int rc;
  int i = 0;

#ifdef ARDUINO
  myWire->beginTransmission(u8Addr);
  myWire->write(u8Register);
  myWire->endTransmission();
  myWire->requestFrom(u8Addr, (uint8_t)iLen);
 // i = myWire->readBytes(pData, iLen);
  while (myWire->available() && i < iLen)
  {
      pData[i++] = myWire->read();
  }
#else
    rc = i2c_master_write_read_device(I2C_NUM_0, u8Addr, &u8Register, 1, pData, iLen, 100);
    i = (rc == ESP_OK);
#endif
  return i;
} /* I2CReadRegister() */
//
// Read N bytes
//
int BBCapTouch::I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen)
{
  int i = 0;

#ifdef ARDUINO
  myWire->requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
     pData[i++] = myWire->read();
  }
#else
    int rc;
    rc = i2c_master_read_from_device(I2C_NUM_0, u8Addr, pData, iLen, 100);
    i = (rc == ESP_OK) ? iLen : 0;
#endif
  return i;
} /* I2CRead() */
//
// Private function to rotate touch samples if the user
// specified a new display orientation
//
void BBCapTouch::fixSamples(TOUCHINFO *pTI)
{
int i, x, y;

   for (i=0; i<pTI->count; i++) {
       switch (_iOrientation) {
           case 90:
               x = pTI->y[i];
               y = _iWidth - 1 - pTI->x[i];
               pTI->x[i] = x;
               pTI->y[i] = y;
               break;
           case 180:
               pTI->x[i] = _iWidth - 1 - pTI->x[i];
               pTI->y[i] = _iHeight - 1 - pTI->y[i];
               break;
           case 270:
               x = _iHeight - 1 - pTI->y[i];
               y = pTI->x[i];
               pTI->x[i] = x;
               pTI->y[i] = y;
               break;
           default: // do nothing
               break;
       }
   }
} /* fixSamples() */

//
// Read the touch points
// returns 0 (none), 1 if touch points are available
// The point count and info is returned in the TOUCHINFO structure
//
int BBCapTouch::getSamples(TOUCHINFO *pTI)
{
uint8_t c, *s, ucTemp[32];
int i, j, rc;
    
    if (!pTI)
       return 0;
    pTI->count = 0;

    if (_iType == CT_TYPE_MXT144) {
       if (!_mxtdata.t44_message_count_address) {
          return 0; // No message offset, so we can't read anything :(
       }
       I2CReadRegister16(_iAddr, _mxtdata.t44_message_count_address, ucTemp, 1);
       j = ucTemp[0]; // object count
       // As each message is read from the sensor, the internal count
       // is decremented. It appears that it can hold 6 messages maximum
       // before you must read them to receive more.
       for (i = 0; i < j; i++) { // read the messages
          I2CReadRegister16(_iAddr, _mxtdata.t5_message_processor_address, ucTemp, MXT_MESSAGE_SIZE); // each message is 6 bytes
          // check report_id
          if (ucTemp[0] >= _mxtdata.t100_first_report_id + 2 &&
            ucTemp[0] < _mxtdata.t100_first_report_id + 2 + 5) {
              uint8_t finger_idx = ucTemp[0] - _mxtdata.t100_first_report_id - 2;
              uint8_t event = ucTemp[1] & 0xf;
              if (finger_idx+1 > pTI->count) pTI->count = finger_idx+1;
              pTI->x[finger_idx] = ucTemp[2] + (ucTemp[3] << 8);
              pTI->y[finger_idx] = ucTemp[4] + (ucTemp[5] << 8);
              if (event == 1 || event == 4) { // move/press event
                  pTI->area[finger_idx] = 50;
              } else if (event == 5) { // release
                  pTI->area[finger_idx] = 0;
              }
           } // if touch report
       } // for each report
       return (pTI->count > 0);
    } // MXT144

    if (_iType == CT_TYPE_AXS15231) {
        uint8_t ucReadCMD[8] = {0xb5,0xab,0xa5,0x5a,0,0,0,0x8};
        I2CWrite(_iAddr, (uint8_t *)ucReadCMD, 8);
        I2CRead(_iAddr, ucTemp, 14); // read up to 2 touch points
        c = ucTemp[1]; // number of touch points
        if (c == 0 || c > 2 || ucTemp[0] != 0) return 0;
        pTI->count = c;
        j = 0; // buffer offset
        for (i=0; i<c; i++) {
             pTI->x[i] = ((ucTemp[j+2] & 0xf) << 8) + ucTemp[j+3];
             pTI->y[i] = ((ucTemp[j+4] & 0xf) << 8) + ucTemp[j+5];
             pTI->area[i] = 1;
             j += 6; 
        }
        if (_iOrientation != 0) fixSamples(pTI);
        return 1;
    } // AXS15231
 
    if (_iType == CT_TYPE_CST226) {
        i = I2CReadRegister(_iAddr, 0, ucTemp, 28); // read the whole block of regs
//      Serial.printf("I2CReadRegister returned %d\n", i);
#ifdef FUTURE
        if (ucTemp[0] == 0x83 && ucTemp[1] == 0x17 && ucTemp[5] == 0x80) {
        // home button pressed
            return 0;
        }
        if (ucTemp[6] != 0xab) return 0;
        if (ucTemp[0] == 0xab) return 0;
        if (ucTemp[5] == 0x80) return 0;
        c = ucTemp[5] & 0x7f;
        if (c > 5 || c == 0) { // invalid point count
           ucTemp[0] = 0;
           ucTemp[1] = 0xab;
           I2CWrite(_iAddr, ucTemp, 2); // reset
           return 0;
        }
#endif
        c = 1; // debug
        pTI->count = c;
       // Serial.printf("count = %d\n", c);
        j = 0;
        for (i=0; i<c; i++) {
           pTI->x[i] = (uint16_t)((ucTemp[j+1] << 4) | ((ucTemp[j+3] >> 4) & 0xf));
           pTI->y[i] = (uint16_t)((ucTemp[j+2] << 4) | (ucTemp[j+3] & 0xf));
           pTI->pressure[i] = ucTemp[j+4];
           j = (i == 0) ? (j+7) : (j+5);
        }
        if (_iOrientation != 0) fixSamples(pTI);
        return 1;
    }

    if (_iType == CT_TYPE_CST820) {
       I2CReadRegister(_iAddr, CST820_TOUCH_REGS+1, ucTemp, 1); // read touch count
       if (ucTemp[0] < 1 || ucTemp[0] > 5) { // something went wrong
           return 0;
       }
       if (ucTemp[0] >= 1) { // touch data available, read it
           pTI->count = ucTemp[0];
           I2CReadRegister(_iAddr, CST820_TOUCH_REGS+2, ucTemp, pTI->count * 6);
           s = ucTemp;
           for (i=0; i<pTI->count; i++) {
              pTI->x[i] = ((s[0] & 0xf) << 8) | s[1];
              pTI->y[i] = ((s[2] & 0xf) << 8) | s[3];
              pTI->area[i] = 1; // no data available
              s += 6;
           }
           if (_iOrientation != 0) fixSamples(pTI);
           return 1;
       }
    }
    if (_iType == CT_TYPE_FT6X36) {
       rc = I2CReadRegister(_iAddr, TOUCH_REG_STATUS, ucTemp, 1); // read touch status
       if (rc == 0) { // something went wrong
           return 0;
       }
       i = ucTemp[0]; // number of touch points available
       if (i >= 1) { // get data
           rc = I2CReadRegister(_iAddr, TOUCH_REG_XH, ucTemp, 6*i); // read X+Y position(s)
           if ((ucTemp[0] & 0x40) == 0 && (ucTemp[2] & 0xf0) != 0xf0) { // finger is down
               pTI->x[0] = ((ucTemp[0] & 0xf) << 8) | ucTemp[1];
               pTI->y[0] = ((ucTemp[2] & 0xf) << 8) | ucTemp[3];
               // get touch pressure and area
               pTI->pressure[0] = ucTemp[4];
               pTI->area[0] = ucTemp[5];
               pTI->count++;
           }
           if (i > 1) { // get second point
               if ((ucTemp[6] & 0x40) == 0 && (ucTemp[8] & 0xf0) != 0xf0) { // finger is down
                   pTI->x[1] = ((ucTemp[6] & 0xf) << 8) | ucTemp[7];
                   pTI->y[1] = ((ucTemp[8] & 0xf) << 8) | ucTemp[9];
                   // get touch pressure and area
                   pTI->pressure[1] = ucTemp[10];
                   pTI->area[1] = ucTemp[11];
                   pTI->count++;
               }
           }
       } // if touch points available
       if (_iOrientation != 0) fixSamples(pTI);
       return 1;
    } else { // GT911
      I2CReadRegister16(_iAddr, GT911_POINT_INFO, ucTemp, 1); // get number of touch points
      i = ucTemp[0] & 0xf; // number of touches
      if (i <= 5 && ucTemp[0] & 0x80) { // if buffer status is good + >= 1 touch points
         ucTemp[0] = (uint8_t)(GT911_POINT_INFO >> 8);
         ucTemp[1] = (uint8_t)GT911_POINT_INFO;
         ucTemp[2] = 0; // clear touch info for next time
         I2CWrite(_iAddr, ucTemp, 3);

         pTI->count = i;
         for (int j=0; j<i; j++) { // read each touch point block
             I2CReadRegister16(_iAddr, GT911_POINT_1 + (j*8), ucTemp, 7);
             pTI->x[j] = ucTemp[1] + (ucTemp[2] << 8);
             pTI->y[j] = ucTemp[3] + (ucTemp[4] << 8);
             pTI->area[j] = ucTemp[5] + (ucTemp[6] << 8);
             pTI->pressure[j] = 0; 
         }
         if (i && _iOrientation != 0) fixSamples(pTI);
         return (i > 0);
      }
    } // GT911
    return 0;
} /* getSamples() */

int BBCapTouch::setOrientation(int iOrientation, int iWidth, int iHeight)
{
    if (iOrientation != 0 && iOrientation != 90 && iOrientation != 180 && iOrientation != 270) {
       return CT_ERROR;
    }
    _iOrientation = iOrientation;
    _iWidth = iWidth;
    _iHeight = iHeight;
    return CT_SUCCESS;
} /* setOrientation() */

int BBCapTouch::sensorType(void)
{
   return _iType;
} /* type() */
