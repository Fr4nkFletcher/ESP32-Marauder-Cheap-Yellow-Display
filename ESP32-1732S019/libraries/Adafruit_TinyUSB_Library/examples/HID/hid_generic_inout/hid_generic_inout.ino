/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example demonstrate HID Generic raw Input & Output.
 * It will receive data from Host (In endpoint) and echo back (Out endpoint).
 * HID Report descriptor use vendor for usage page (using template TUD_HID_REPORT_DESC_GENERIC_INOUT)
 *
 * There are 2 ways to test the sketch
 * 1. Using nodejs
 * - Install nodejs and npm to your PC
 *
 * - Install excellent node-hid (https://github.com/node-hid/node-hid) by
 *   $ npm install node-hid
 *
 * - Run provided hid test script
 *   $ node hid_test.js
 *
 * 2. Using python
 * - Install `hid` package (https://pypi.org/project/hid/) by
 *   $ pip install hid
 *
 * - hid package replies on hidapi (https://github.com/libusb/hidapi) for backend,
 *   which already available in Linux. However on windows, you may need to download its dlls from their release page and
 *   copy it over to folder where python is installed.
 *
 * - Run provided hid test script to send and receive data to this device.
 *   $ python3 hid_test.py
 */

#include "Adafruit_TinyUSB.h"

// HID report descriptor using TinyUSB's template
// Generic In Out with 64 bytes report (max)
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(64)
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// the setup function runs once when you press reset or power the board
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // Notes: following commented-out functions has no affect on ESP32
  usb_hid.enableOutEndpoint(true);
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setStringDescriptor("TinyUSB HID Generic");
  usb_hid.setReportCallback(get_report_callback, set_report_callback);
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial.println("Adafruit TinyUSB HID Generic In Out example");
}

void loop() {
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
  // not used in this example
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;
  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
  // This example doesn't use multiple report and report ID
  (void) report_id;
  (void) report_type;

  // echo back anything we received from host
  usb_hid.sendReport(0, buffer, bufsize);
}
