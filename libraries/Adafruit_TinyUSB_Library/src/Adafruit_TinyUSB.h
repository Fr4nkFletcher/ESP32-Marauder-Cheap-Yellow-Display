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

#ifndef ADAFRUIT_TINYUSB_H_
#define ADAFRUIT_TINYUSB_H_

// ESP32 out-of-sync
#ifdef ARDUINO_ARCH_ESP32
#include "arduino/ports/esp32/tusb_config_esp32.h"
#endif

#include "tusb_option.h"

// Device
#if CFG_TUD_ENABLED

#include "arduino/Adafruit_USBD_Device.h"

#if CFG_TUD_CDC
#include "arduino/Adafruit_USBD_CDC.h"
#endif

#if CFG_TUD_HID
#include "arduino/hid/Adafruit_USBD_HID.h"
#endif

#if CFG_TUD_MIDI
#include "arduino/midi/Adafruit_USBD_MIDI.h"
#endif

#if CFG_TUD_MSC
#include "arduino/msc/Adafruit_USBD_MSC.h"
#endif

#if CFG_TUD_VENDOR
#include "arduino/webusb/Adafruit_USBD_WebUSB.h"
#endif

#if CFG_TUD_VIDEO
#include "arduino/video/Adafruit_USBD_Video.h"
#endif

// Initialize device hardware, stack, also Serial as CDC
// Wrapper for TinyUSBDevice.begin(rhport)
void TinyUSB_Device_Init(uint8_t rhport);

#endif

// Host
#if CFG_TUH_ENABLED

#include "arduino/Adafruit_USBH_Host.h"

#if CFG_TUH_CDC
#include "arduino/cdc/Adafruit_USBH_CDC.h"
#endif

#if CFG_TUH_MSC
#include "arduino/msc/Adafruit_USBH_MSC.h"
#endif

#endif

#endif /* ADAFRUIT_TINYUSB_H_ */
