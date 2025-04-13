/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
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

#ifndef ADAFRUIT_USBH_HOST_H_
#define ADAFRUIT_USBH_HOST_H_

#include "Adafruit_USBD_Interface.h"
#include "tusb.h"
#include <SPI.h>

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-tinyusb.h"
#endif

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
extern "C" {
void tuh_max3421_spi_cs_api(uint8_t rhport, bool active);
bool tuh_max3421_spi_xfer_api(uint8_t rhport, uint8_t const *tx_buf,
                              uint8_t *rx_buf, size_t xfer_bytes);
void tuh_max3421_int_api(uint8_t rhport, bool enabled);
}

class Adafruit_USBH_Host {
private:
  SPIClass *_spi;
  int8_t _cs;
  int8_t _intr;

  // for esp32 or using softwareSPI
  int8_t _sck, _mosi, _miso;

public:
  // constructor for using MAX3421E (host shield)
  Adafruit_USBH_Host(SPIClass *spi, int8_t cs, int8_t intr);
  Adafruit_USBH_Host(SPIClass *spi, int8_t sck, int8_t mosi, int8_t miso,
                     int8_t cs, int8_t intr);

  uint8_t max3421_readRegister(uint8_t reg, bool in_isr);
  bool max3421_writeRegister(uint8_t reg, uint8_t data, bool in_isr);
  bool max3421_writeIOPINS1(uint8_t data, bool in_isr) {
    enum { IOPINS1_ADDR = 20u << 3 }; // 0xA0
    return max3421_writeRegister(IOPINS1_ADDR, data, in_isr);
  }
  bool max3421_writeIOPINS2(uint8_t data, bool in_isr) {
    enum { IOPINS2_ADDR = 21u << 3 }; // 0xA8
    return max3421_writeRegister(IOPINS2_ADDR, data, in_isr);
  }

private:
  friend void tuh_max3421_spi_cs_api(uint8_t rhport, bool active);
  friend bool tuh_max3421_spi_xfer_api(uint8_t rhport, uint8_t const *tx_buf,
                                       uint8_t *rx_buf, size_t xfer_bytes);
  friend void tuh_max3421_int_api(uint8_t rhport, bool enabled);
#else

class Adafruit_USBH_Host {
#endif

public:
  // default constructor
  Adafruit_USBH_Host(void);

  bool configure(uint8_t rhport, uint32_t cfg_id, const void *cfg_param);

#ifdef ARDUINO_ARCH_RP2040
  bool configure_pio_usb(uint8_t rhport, const void *cfg_param);
#endif

  bool begin(uint8_t rhport);
  void task(uint32_t timeout_ms = UINT32_MAX, bool in_isr = false);

  //------------- internal usage -------------//
  static Adafruit_USBH_Host *_instance;

private:
  uint8_t _rhport;
};

#endif
