/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include "Adafruit_TinyUSB.h"

/* This sketch demonstrates USB CDC Serial echo (convert to upper case) using SerialTinyUSB which
 * is available for both core with built-in USB support and without.
 * Note: on core with built-in support Serial is alias to SerialTinyUSB
 */

void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
}

void loop() {
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  uint8_t buf[64];
  uint32_t count = 0;
  while (SerialTinyUSB.available()) {
    buf[count++] = (uint8_t) toupper(SerialTinyUSB.read());
  }

  if (count) {
    SerialTinyUSB.write(buf, count);
  }
}
