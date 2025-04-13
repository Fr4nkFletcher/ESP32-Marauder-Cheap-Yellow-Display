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

#ifndef ADAFRUIT_USBD_DEVICE_H_
#define ADAFRUIT_USBD_DEVICE_H_

#include "Adafruit_USBD_Interface.h"

#include "../tusb.h" // use relative path to prevent ESP32 out-of-sync issue

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-tinyusb.h"
#endif

class Adafruit_USBD_Device {
private:
  enum { STRING_DESCRIPTOR_MAX = 12 };

  // Device descriptor
  tusb_desc_device_t _desc_device __attribute__((aligned(4)));

  // Configuration descriptor
  uint8_t *_desc_cfg;
  uint8_t _desc_cfg_buffer[256];
  uint16_t _desc_cfg_len;
  uint16_t _desc_cfg_maxlen;

  uint8_t _itf_count;

  uint8_t _epin_count;
  uint8_t _epout_count;

  // String descriptor
  const char *_desc_str_arr[STRING_DESCRIPTOR_MAX];
  uint8_t _desc_str_count;
  uint16_t _desc_str[32 + 1]; // up to 32 unicode characters with headers

public:
  Adafruit_USBD_Device(void);

  //------------- Device descriptor -------------//

  // Set VID, PID
  void setID(uint16_t vid, uint16_t pid);

  // Set bcdUSB version e.g 1.0, 2.0, 2.1
  void setVersion(uint16_t bcd);

  // Set bcdDevice version
  void setDeviceVersion(uint16_t bcd);

  //------------- Configuration descriptor -------------//

  // Add a new interface
  bool addInterface(Adafruit_USBD_Interface &itf);

  // Clear/Reset configuration descriptor
  void clearConfiguration(void);

  // Provide user buffer for configuration descriptor, if total length > 256
  void setConfigurationBuffer(uint8_t *buf, uint32_t buflen);

  // Allocate a new interface number
  uint8_t allocInterface(uint8_t count = 1) {
    uint8_t ret = _itf_count;
    _itf_count += count;
    return ret;
  }

  uint8_t allocEndpoint(uint8_t in) {
    uint8_t ret = in ? (0x80 | _epin_count++) : _epout_count++;
#if defined(ARDUINO_ARCH_ESP32) && ARDUINO_USB_CDC_ON_BOOT && !ARDUINO_USB_MODE
    // ESP32 reserves 0x03, 0x84, 0x85 for CDC Serial
    if (ret == 0x03) {
      ret = _epout_count++;
    } else if (ret == 0x84 || ret == 0x85) {
      // Note: ESP32 does not have this much of EP IN
      _epin_count = 6;
      ret = 0x86;
    }
#endif
    return ret;
  }

  //------------- String descriptor -------------//
  void setLanguageDescriptor(uint16_t language_id);
  void setManufacturerDescriptor(const char *s);
  void setProductDescriptor(const char *s);
  void setSerialDescriptor(const char *s);
  uint8_t getSerialDescriptor(uint16_t *serial_utf16);

  uint8_t addStringDescriptor(const char *s);

  //------------- Control -------------//

  bool begin(uint8_t rhport = 0);
  bool isInitialized(uint8_t rhport = 0);
  void task(void);

  // physical disable/enable pull-up
  bool detach(void);
  bool attach(void);

  //------------- status -------------//
  bool mounted(void);
  bool suspended(void);
  bool ready(void);
  bool remoteWakeup(void);

private:
  uint16_t const *descriptor_string_cb(uint8_t index, uint16_t langid);

  friend uint8_t const *tud_descriptor_device_cb(void);
  friend uint8_t const *tud_descriptor_configuration_cb(uint8_t index);
  friend uint16_t const *tud_descriptor_string_cb(uint8_t index,
                                                  uint16_t langid);
};

extern Adafruit_USBD_Device TinyUSBDevice;

// USBDevice has a high chance to conflict with other usb stack
// only define if supported BSP
#ifdef USE_TINYUSB
#define USBDevice TinyUSBDevice
#endif

#endif /* ADAFRUIT_USBD_DEVICE_H_ */
