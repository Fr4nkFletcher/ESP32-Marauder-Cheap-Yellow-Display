/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ADAFRUIT_USBD_I2C_H_
#define ADAFRUIT_USBD_I2C_H_

#include "Adafruit_TinyUSB.h"
#include "Wire.h"

/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO        0
#define CMD_GET_FUNC    1
#define CMD_SET_DELAY   2
#define CMD_GET_STATUS  3

#define CMD_I2C_IO        4
#define CMD_I2C_IO_BEGIN  (1<<0) // flag for I2C_IO
#define CMD_I2C_IO_END    (1<<1) // flag for I2C_IO

/* linux kernel flags */
#define I2C_M_TEN            0x10  /* we have a ten bit chip address */
#define I2C_M_RD             0x01
#define I2C_M_NOSTART        0x4000
#define I2C_M_REV_DIR_ADDR   0x2000
#define I2C_M_IGNORE_NAK     0x1000
#define I2C_M_NO_RD_ACK      0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C                     0x00000001
#define I2C_FUNC_10BIT_ADDR              0x00000002 /* required for I2C_M_TEN */
#define I2C_FUNC_PROTOCOL_MANGLING       0x00000004 /* required for I2C_M_IGNORE_NAK etc. */
#define I2C_FUNC_SMBUS_PEC               0x00000008
#define I2C_FUNC_NOSTART                 0x00000010 /* required for I2C_M_NOSTART */
#define I2C_FUNC_SLAVE                   0x00000020
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL   0x00008000 /* SMBus 2.0 or later */
#define I2C_FUNC_SMBUS_QUICK             0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE         0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE        0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA    0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA   0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA    0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA   0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL         0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA   0x01000000 /* required for I2C_M_RECV_LEN */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA  0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK    0x04000000 /* I2C-like block xfer   */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK   0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_HOST_NOTIFY       0x10000000 /* SMBus 2.0 or later */

#define I2C_FUNC_SMBUS_BYTE       (I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE)
#define I2C_FUNC_SMBUS_BYTE_DATA  (I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_BYTE_DATA)
#define I2C_FUNC_SMBUS_WORD_DATA  (I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA)
#define I2C_FUNC_SMBUS_BLOCK_DATA (I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)
#define I2C_FUNC_SMBUS_I2C_BLOCK  (I2C_FUNC_SMBUS_READ_I2C_BLOCK | I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)

#define I2C_FUNC_SMBUS_EMUL    (I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA | \
                                I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_PROC_CALL | I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                                I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_PEC)

/* if I2C_M_RECV_LEN is also supported */
#define I2C_FUNC_SMBUS_EMUL_ALL (I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_BLOCK_PROC_CALL)

#define I2C_STATUS_IDLE 0
#define I2C_STATUS_ACK  1
#define I2C_STATUS_NAK  2

class Adafruit_USBD_I2C : public Adafruit_USBD_Interface {
public:
  Adafruit_USBD_I2C(TwoWire* wire);
  bool begin(uint8_t* buffer, size_t bufsize);

  bool handleControlTransfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request);

  // from Adafruit_USBD_Interface
  virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t* buf, uint16_t bufsize);

private:
  TwoWire* _wire;
  uint8_t _state;
  uint32_t _functionality;
  uint8_t* _buf;
  uint16_t _bufsize;

  uint16_t i2c_write(uint8_t addr, uint8_t const* buf, uint16_t len, bool stop_bit);
  uint16_t i2c_read(uint8_t addr, uint8_t* buf, uint16_t len, bool stop_bit);
};

#endif
