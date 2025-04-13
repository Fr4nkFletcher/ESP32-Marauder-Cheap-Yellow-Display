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

/* This sketch demonstrates multiple USB HID interfaces. Pressing the button will
 * - mouse toward bottom right of monitor
 * - send 'a' key
 *
 * Depending on the board, the button pin
 * and its active state (when pressed) are different
 */
#if defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(ARDUINO_NRF52840_CIRCUITPLAY)
const int pin = 4; // Left Button
bool activeState = true;

#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
const int pin = BUTTON_DOWN;
bool activeState = true;

#elif defined PIN_BUTTON1
const int pin = PIN_BUTTON1;
bool activeState = false;

#elif defined PIN_BUTTON
const int pin = PIN_BUTTON;
bool activeState = false;

#elif defined(ARDUINO_ARCH_ESP32)
const int pin = 0;
bool activeState = false;

#elif defined(ARDUINO_ARCH_RP2040)
const int pin = D0;
bool activeState = false;
#else

const int pin = A0;
bool activeState = false;
#endif

// HID report descriptor using TinyUSB's template
uint8_t const desc_keyboard_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t const desc_mouse_report[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID objects
Adafruit_USBD_HID usb_keyboard;
Adafruit_USBD_HID usb_mouse;

// the setup function runs once when you press reset or power the board
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // HID Keyboard
  usb_keyboard.setPollInterval(2);
  usb_keyboard.setBootProtocol(HID_ITF_PROTOCOL_KEYBOARD);
  usb_keyboard.setReportDescriptor(desc_keyboard_report, sizeof(desc_keyboard_report));
  usb_keyboard.setStringDescriptor("TinyUSB HID Keyboard");
  usb_keyboard.begin();

  // HID Mouse
  usb_mouse.setPollInterval(2);
  usb_mouse.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
  usb_mouse.setReportDescriptor(desc_mouse_report, sizeof(desc_mouse_report));
  usb_mouse.setStringDescriptor("TinyUSB HID Keyboard");
  usb_mouse.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Set up button, pullup opposite to active state
  pinMode(pin, activeState ? INPUT_PULLDOWN : INPUT_PULLUP);

  Serial.println("Adafruit TinyUSB HID Composite example");
}

void process_hid() {
  // Whether button is pressed
  bool btn_pressed = (digitalRead(pin) == activeState);

  // Remote wakeup
  if (TinyUSBDevice.suspended() && btn_pressed) {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    TinyUSBDevice.remoteWakeup();
  }

  /*------------- Mouse -------------*/
  if (usb_mouse.ready() && btn_pressed) {
    int8_t const delta = 5;
    usb_mouse.mouseMove(0, delta, delta); // right + down
  }

  /*------------- Keyboard -------------*/
  if (usb_keyboard.ready()) {
    // use to send key release report
    static bool has_key = false;

    if (btn_pressed) {
      uint8_t keycode[6] = {0};
      keycode[0] = HID_KEY_A;

      usb_keyboard.keyboardReport(0, 0, keycode);

      has_key = true;
    } else {
      // send empty key report if previously has key pressed
      if (has_key) usb_keyboard.keyboardRelease(0);
      has_key = false;
    }
  }
}

void loop() {
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }

  // poll gpio once each 10 ms
  static uint32_t ms = 0;
  if (millis() - ms > 10) {
    ms = millis();
    process_hid();
  }
}