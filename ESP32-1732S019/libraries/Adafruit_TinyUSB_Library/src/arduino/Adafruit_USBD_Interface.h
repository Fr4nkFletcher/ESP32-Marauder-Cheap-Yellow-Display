/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Ha Thach (tinyusb.org) for Adafruit Industries
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

#ifndef ADAFRUIT_USBD_INTERFACE_H_
#define ADAFRUIT_USBD_INTERFACE_H_

#include <stddef.h>
#include <stdint.h>

#if defined(ARDUINO_ARCH_CH32) || defined(CH32V20x) || defined(CH32V30x)
// HACK: required for ch32 core version 1.0.4 or prior, removed when 1.0.5 is
// released
extern "C" void yield(void);
#endif

class Adafruit_USBD_Interface {
protected:
  uint8_t _strid;

public:
  Adafruit_USBD_Interface(void) { _strid = 0; }

  // Get Interface Descriptor
  // Fill the descriptor (if buf is not NULL) and return its length
  virtual uint16_t getInterfaceDescriptor(uint8_t itfnum_deprecated,
                                          uint8_t *buf, uint16_t bufsize) = 0;
  // Get Interface Descriptor Length
  uint16_t getInterfaceDescriptorLen() {
    return getInterfaceDescriptor(0, NULL, 0);
  }

  void setStringDescriptor(const char *str);
};

#endif
