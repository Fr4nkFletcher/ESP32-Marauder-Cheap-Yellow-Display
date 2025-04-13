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

/* This sketch demonstrates USB CDC Serial can be dropped by simply
 * call Serial.end() within setup(). This must be called before any
 * other USB interfaces (MSC / HID) begin to have a clean configuration
 *
 * Note: this will cause device to loose the touch1200 and require
 * user manual interaction to put device into bootloader/DFU mode.
 */

int led = LED_BUILTIN;

void setup()
{
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // clear configuration will remove all USB interfaces including CDC (Serial)
  TinyUSBDevice.clearConfiguration();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  pinMode(led, OUTPUT);
}

void loop()
{
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // toggle LED
  static uint32_t ms = 0;
  static uint8_t led_state = 0;
  if (millis() - ms > 1000) {
    ms = millis();
    digitalWrite(LED_BUILTIN, 1-led_state);
  }
}
