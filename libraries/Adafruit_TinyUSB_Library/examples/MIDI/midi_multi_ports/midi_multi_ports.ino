/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

// This sketch is enumerated as USB MIDI device with multiple ports
// and how to set their name

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

// USB MIDI object with 3 ports
Adafruit_USBD_MIDI usb_midi(3);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // Set name for each cable, must be done before usb_midi.begin()
  usb_midi.setCableName(1, "Keyboard");
  usb_midi.setCableName(2, "Drum Pads");
  usb_midi.setCableName(3, "Lights");
  usb_midi.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

void loop() {
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
