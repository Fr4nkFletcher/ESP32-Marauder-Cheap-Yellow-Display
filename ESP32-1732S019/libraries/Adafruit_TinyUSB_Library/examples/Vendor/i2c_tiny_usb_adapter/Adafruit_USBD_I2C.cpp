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

/*
 * Reference:
 * - https://github.com/torvalds/linux/blob/master/drivers/i2c/busses/i2c-tiny-usb.c
 * - https://github.com/harbaum/I2C-Tiny-USB
 *
 * Requirement:
 * - Install i2c-tools with
 *    sudo apt install i2c-tools
 *
 * How to test example:
 * - Compile and flash this sketch on your board with an i2c device, it should enumerated as
 *    ID 1c40:0534 EZPrototypes i2c-tiny-usb interface
 *
 * - Run "i2cdetect -l" to find our bus ID e.g
 *    i2c-8	i2c       	i2c-tiny-usb at bus 003 device 039	I2C adapter
 *
 * - Run "i2cdetect -y 8" to scan for on-board device (8 is the above bus ID)
 *         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
      00:                         -- -- -- -- -- -- -- --
      10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
      70: -- -- -- -- -- -- -- 77

   - You can then interact with sensor using following commands:
     i2cget i2cset i2cdump i2ctransfer or using any driver/tools that work on i2c device.
 */

#include "Adafruit_USBD_I2C.h"

Adafruit_USBD_I2C::Adafruit_USBD_I2C(TwoWire* wire) {
  _wire = wire;
  _buf = NULL;
  _bufsize = 0; // not used to verify length yet
  _state = I2C_STATUS_IDLE;
  _functionality = 0x8eff0001; // check out _functionality_* defines
  setStringDescriptor("I2C Interface");
}

uint16_t Adafruit_USBD_I2C::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t* buf, uint16_t bufsize) {
  uint8_t itfnum = 0;
  uint8_t ep_in = 0;
  uint8_t ep_out = 0;
  (void) itfnum_deprecated;

  // null buffer is used to get the length of descriptor only
  if (buf) {
    itfnum = TinyUSBDevice.allocInterface(1);
    ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
  }

  uint8_t const desc[] = { TUD_VENDOR_DESCRIPTOR(itfnum, _strid, ep_out, ep_in, 64) };
  uint16_t const len = sizeof(desc);

  if (buf) {
    if (bufsize < len) {
      return 0;
    }
    memcpy(buf, desc, len);
  }

  return len;
}

bool Adafruit_USBD_I2C::begin(uint8_t* buffer, size_t bufsize) {
  _buf = buffer;
  _bufsize = (uint16_t) bufsize;

  if (!_wire || !_buf || !_bufsize) return false;

  // needed to identify as a device for i2c_tiny_usb (EZPrototypes VID/PID)
  TinyUSBDevice.setID(0x1c40, 0x0534);
  if (!TinyUSBDevice.addInterface(*this)) return false;

  _wire->begin();
  return true;
}

uint16_t Adafruit_USBD_I2C::i2c_read(uint8_t addr, uint8_t* buf, uint16_t len, bool stop_bit)
{
  uint16_t const rd_count = (uint16_t) _wire->requestFrom(addr, len, stop_bit);

  _state = (len && !rd_count) ? I2C_STATUS_NAK : I2C_STATUS_ACK;

  // Serial.printf("I2C Read: addr = 0x%02X, len = %u, rd_count %u bytes, status = %u\r\n", addr, len, rd_count, i2c_state);

  for(uint16_t i = 0; i <rd_count; i++)
  {
    buf[i] = (uint8_t) _wire->read();
  }

  return rd_count;
}

uint16_t Adafruit_USBD_I2C::i2c_write(uint8_t addr, uint8_t const* buf, uint16_t len, bool stop_bit)
{
  _wire->beginTransmission(addr);
  uint16_t wr_count = (uint16_t) _wire->write(buf, len);
  uint8_t const sts = _wire->endTransmission(stop_bit);

  _state = (sts == 0) ? I2C_STATUS_ACK : I2C_STATUS_NAK;

  // Serial.printf("I2C Write: addr = 0x%02X, len = %u, wr_count = %u, status = %u\r\n", addr, len, wr_count, i2c_state);

  return wr_count;
}

bool Adafruit_USBD_I2C::handleControlTransfer(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
  uint8_t const cmd = request->bRequest;

  if ( stage == CONTROL_STAGE_SETUP )
  {
    switch ( cmd )
    {
      case CMD_ECHO:
        // echo
        return tud_control_xfer(rhport, request, (void*) &request->wValue, sizeof(request->wValue));

      case CMD_GET_FUNC:
        // capabilities
        return tud_control_xfer(rhport, request, (void*) &_functionality, sizeof(_functionality));

      case CMD_SET_DELAY:
        if ( request->wValue == 0 )
        {
          _wire->setClock(115200);
        }
        else
        {
          int baudrate = 1000000 / request->wValue;
          if ( baudrate > 400000 ) baudrate = 400000;
          _wire->setClock(baudrate);
        }
        return tud_control_status(rhport, request);

      case CMD_GET_STATUS:
        return tud_control_xfer(rhport, request, (void*) &_state, sizeof(_state));

      case CMD_I2C_IO:
      case CMD_I2C_IO | CMD_I2C_IO_BEGIN:
      case CMD_I2C_IO | CMD_I2C_IO_END:
      case CMD_I2C_IO | CMD_I2C_IO_BEGIN | CMD_I2C_IO_END:
      {
        uint8_t const addr = (uint8_t) request->wIndex;
        // uint16_t const flags = request->wValue;
        uint16_t const len = tu_min16(request->wLength, _bufsize);
        bool const stop_bit = (cmd & CMD_I2C_IO_END) ? true : false;

        if (request->bmRequestType_bit.direction == TUSB_DIR_OUT)
        {
          if (len == 0)
          {
            // zero write: do it here since there will be no data stage for len = 0
            i2c_write(addr, _buf, len, stop_bit);
          }
          return tud_control_xfer(rhport, request, _buf, len);
        }else
        {
          uint16_t const rd_count = i2c_read(addr, _buf, len, stop_bit);
          return tud_control_xfer(rhport, request, rd_count ? _buf : NULL, rd_count);
        }
      }
      break;

      default: return true;
    }
  }
  else if ( stage == CONTROL_STAGE_DATA )
  {
    switch ( cmd )
    {
      case CMD_I2C_IO:
      case CMD_I2C_IO | CMD_I2C_IO_BEGIN:
      case CMD_I2C_IO | CMD_I2C_IO_END:
      case CMD_I2C_IO | CMD_I2C_IO_BEGIN | CMD_I2C_IO_END:
        if (request->bmRequestType_bit.direction == TUSB_DIR_OUT)
        {
          uint8_t const addr = (uint8_t) request->wIndex;
          // uint16_t const flags = request->wValue;
          uint16_t const len = tu_min16(request->wLength, _bufsize);
          bool const stop_bit = (cmd & CMD_I2C_IO_END) ? true : false;

          i2c_write(addr, _buf, len, stop_bit);
        }
        return true;

      default: return true;
    }
  }
  else
  {
    // CONTROL_STAGE_STATUS
    return true;
  }
}


