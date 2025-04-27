/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Ha Thach (tinyusb.org) for Adafruit Industries
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

// ESP32 out-of-sync
#ifdef ARDUINO_ARCH_ESP32
#include "arduino/ports/esp32/tusb_config_esp32.h"
#endif

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUH_MSC

#include "tusb.h"

#include "Adafruit_USBH_CDC.h"

Adafruit_USBH_CDC::Adafruit_USBH_CDC(void)
#ifdef ARDUINO_ARCH_ESP32
    : HardwareSerial(0)
#endif
{
  _idx = TUSB_INDEX_INVALID_8;
}

void Adafruit_USBH_CDC::begin(unsigned long baudrate) {
  // default to index 0 when begin
  if (_idx == TUSB_INDEX_INVALID_8) {
    _idx = 0;
  }

  _baud = baudrate;
  if (_baud == 0) {
    _baud = 115200; // default, backward compatible with previous API begin(0)
  }

  // if already mounted, this will set baudrate
  if (mounted()) {
    setBaudrate(_baud);
  }
}

void Adafruit_USBH_CDC::begin(unsigned long baudrate, uint16_t config) {
  (void)config; // TODO support line coding later
  begin(baudrate);
}

void Adafruit_USBH_CDC::end(void) { _idx = TUSB_INDEX_INVALID_8; }

bool Adafruit_USBH_CDC::mount(uint8_t idx) {
  _idx = idx;

  uint32_t local_baud = baud();
  if (local_baud != _baud) {
    return setBaudrate(_baud);
  }

  return true;
}

void Adafruit_USBH_CDC::umount(uint8_t idx) {
  if (_idx == idx) {
    _idx = TUSB_INDEX_INVALID_8;
  }
}

bool Adafruit_USBH_CDC::connected(void) { return tuh_cdc_connected(_idx); }

bool Adafruit_USBH_CDC::mounted(void) { return tuh_cdc_mounted(_idx); }
uint32_t Adafruit_USBH_CDC::baud() {
  cdc_line_coding_t line_coding;
  if (!tuh_cdc_get_local_line_coding(_idx, &line_coding)) {
    return 0;
  }
  return line_coding.bit_rate;
}

//--------------------------------------------------------------------+
// Control API
//--------------------------------------------------------------------+

bool Adafruit_USBH_CDC::setDtrRts(bool dtr, bool rts, tuh_xfer_cb_t complete_cb,
                                  uintptr_t user_data) {
  if (!tuh_cdc_mounted(_idx)) {
    return false;
  }

  uint16_t const line_state = (dtr ? CDC_CONTROL_LINE_STATE_DTR : 0) |
                              (rts ? CDC_CONTROL_LINE_STATE_RTS : 0);

  return tuh_cdc_set_control_line_state(_idx, line_state, complete_cb,
                                        user_data);
}

bool Adafruit_USBH_CDC::setBaudrate(uint32_t baudrate,
                                    tuh_xfer_cb_t complete_cb,
                                    uintptr_t user_data) {
  if (!tuh_cdc_mounted(_idx)) {
    return false;
  }

  if (baud() == baudrate) {
    // skip if already matched
    return true;
  }

  return tuh_cdc_set_baudrate(_idx, baudrate, complete_cb, user_data);
}

//--------------------------------------------------------------------+
// Stream API
//--------------------------------------------------------------------+

int Adafruit_USBH_CDC::available(void) {
  return (int)tuh_cdc_read_available(_idx);
}

int Adafruit_USBH_CDC::peek(void) {
  uint8_t ch;
  return tuh_cdc_peek(_idx, &ch) ? (int)ch : -1;
}

int Adafruit_USBH_CDC::read(void) {
  uint8_t ch;
  return read(&ch, 1) ? (int)ch : -1;
}

size_t Adafruit_USBH_CDC::read(uint8_t *buffer, size_t size) {
  return tuh_cdc_read(_idx, buffer, size);
}

void Adafruit_USBH_CDC::flush(void) { (void)tuh_cdc_write_flush(_idx); }

size_t Adafruit_USBH_CDC::write(uint8_t ch) { return write(&ch, 1); }

size_t Adafruit_USBH_CDC::write(const uint8_t *buffer, size_t size) {
  size_t remain = size;
  while (remain && tuh_cdc_mounted(_idx)) {
    size_t wrcount = tuh_cdc_write(_idx, buffer, remain);
    remain -= wrcount;
    buffer += wrcount;

    // Write FIFO is full, run host task while wait for space become available
    if (remain) {
      tuh_task();
    }
  }

  return size - remain;

  return tuh_cdc_write(_idx, buffer, size);
}

int Adafruit_USBH_CDC::availableForWrite(void) {
  return (int)tuh_cdc_write_available(_idx);
}

#endif
