/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This sketch demonstrates WebUSB as web serial with browser with WebUSB support (e.g Chrome).
 * After enumerated successfully, Browser will pop-up notification
 * with URL to landing page, click on it to test
 *  - Click "Connect" and select device, When connected the on-board LED will litted up.
 *  - Any charters received from either webusb/Serial will be echo back to webusb and Serial
 *  
 * Note: 
 * - The WebUSB landing page notification is currently disabled in Chrome 
 * on Windows due to Chromium issue 656702 (https://crbug.com/656702). You have to 
 * go to landing page (below) to test
 * 
 * - On Windows 7 and prior: You need to use Zadig tool to manually bind the 
 * WebUSB interface with the WinUSB driver for Chrome to access. From windows 8 and 10, this
 * is done automatically by firmware.
 */

#include "Adafruit_TinyUSB.h"

// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;

// Landing Page: scheme (0: http, 1: https), url
// Page source can be found at https://github.com/hathach/tinyusb-webusb-page/tree/main/webusb-serial
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "example.tinyusb.org/webusb-serial/index.html");

int led_pin = LED_BUILTIN;

// the setup function runs once when you press reset or power the board
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  Serial.begin(115200);

  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
  //usb_web.setStringDescriptor("TinyUSB WebUSB");
  usb_web.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // wait until device mounted
  while (!TinyUSBDevice.mounted()) delay(1);

  Serial.println("TinyUSB WebUSB Serial example");
}

// function to echo to both Serial and WebUSB
void echo_all(uint8_t buf[], uint32_t count) {
  if (usb_web.connected()) {
    usb_web.write(buf, count);
    usb_web.flush();
  }

  if (Serial) {
    for (uint32_t i = 0; i < count; i++) {
      Serial.write(buf[i]);
      if (buf[i] == '\r') Serial.write('\n');
    }
    Serial.flush();
  }
}

void loop() {
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  uint8_t buf[64];
  uint32_t count;

  // From Serial to both Serial & webUSB
  if (Serial.available()) {
    count = Serial.read(buf, 64);
    echo_all(buf, count);
  }

  // from WebUSB to both Serial & webUSB
  if (usb_web.available()) {
    count = usb_web.read(buf, 64);
    echo_all(buf, count);
  }
}

void line_state_callback(bool connected) {
  digitalWrite(led_pin, connected);

  if (connected) {
    usb_web.println("WebUSB interface connected !!");
    usb_web.flush();
  }
}
