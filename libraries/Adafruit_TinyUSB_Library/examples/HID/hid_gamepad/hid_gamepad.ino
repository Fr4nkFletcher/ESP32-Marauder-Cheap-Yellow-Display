/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2021 NeKuNeKo for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include "Adafruit_TinyUSB.h"

/* This sketch demonstrates USB HID gamepad use.
 * This sketch is only valid on boards which have native USB support
 * and compatibility with Adafruit TinyUSB library. 
 * For example SAMD21, SAMD51, nRF52840.
 * 
 * Make sure you select the TinyUSB USB stack if you have a SAMD board.
 * You can test the gamepad on a Windows system by pressing WIN+R, writing Joy.cpl and pressing Enter.
 */

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
hid_gamepad_report_t gp;

void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial.println("Adafruit TinyUSB HID Gamepad example");
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

//  // Remote wakeup
//  if ( TinyUSBDevice.suspended() && btn )
//  {
//    // Wake up host if we are in suspend mode
//    // and REMOTE_WAKEUP feature is enabled by host
//    TinyUSBDevice.remoteWakeup();
//  }

  if (!usb_hid.ready()) return;

  // Reset buttons
  Serial.println("No pressing buttons");
  gp.x = 0;
  gp.y = 0;
  gp.z = 0;
  gp.rz = 0;
  gp.rx = 0;
  gp.ry = 0;
  gp.hat = 0;
  gp.buttons = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Hat/DPAD UP
  Serial.println("Hat/DPAD UP");
  gp.hat = 1; // GAMEPAD_HAT_UP;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD UP RIGHT
  Serial.println("Hat/DPAD UP RIGHT");
  gp.hat = 2; // GAMEPAD_HAT_UP_RIGHT;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD RIGHT
  Serial.println("Hat/DPAD RIGHT");
  gp.hat = 3; // GAMEPAD_HAT_RIGHT;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD DOWN RIGHT
  Serial.println("Hat/DPAD DOWN RIGHT");
  gp.hat = 4; // GAMEPAD_HAT_DOWN_RIGHT;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD DOWN
  Serial.println("Hat/DPAD DOWN");
  gp.hat = 5; // GAMEPAD_HAT_DOWN;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD DOWN LEFT
  Serial.println("Hat/DPAD DOWN LEFT");
  gp.hat = 6; // GAMEPAD_HAT_DOWN_LEFT;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD LEFT
  Serial.println("Hat/DPAD LEFT");
  gp.hat = 7; // GAMEPAD_HAT_LEFT;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD UP LEFT
  Serial.println("Hat/DPAD UP LEFT");
  gp.hat = 8; // GAMEPAD_HAT_UP_LEFT;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Hat/DPAD CENTER
  Serial.println("Hat/DPAD CENTER");
  gp.hat = 0; // GAMEPAD_HAT_CENTERED;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Joystick 1 UP
  Serial.println("Joystick 1 UP");
  gp.x = 0;
  gp.y = -127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 1 DOWN
  Serial.println("Joystick 1 DOWN");
  gp.x = 0;
  gp.y = 127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 1 RIGHT
  Serial.println("Joystick 1 RIGHT");
  gp.x = 127;
  gp.y = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 1 LEFT
  Serial.println("Joystick 1 LEFT");
  gp.x = -127;
  gp.y = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 1 CENTER
  Serial.println("Joystick 1 CENTER");
  gp.x = 0;
  gp.y = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Joystick 2 UP
  Serial.println("Joystick 2 UP");
  gp.z = 0;
  gp.rz = 127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 2 DOWN
  Serial.println("Joystick 2 DOWN");
  gp.z = 0;
  gp.rz = -127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 2 RIGHT
  Serial.println("Joystick 2 RIGHT");
  gp.z = 127;
  gp.rz = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 2 LEFT
  Serial.println("Joystick 2 LEFT");
  gp.z = -127;
  gp.rz = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Joystick 2 CENTER
  Serial.println("Joystick 2 CENTER");
  gp.z = 0;
  gp.rz = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Analog Trigger 1 UP
  Serial.println("Analog Trigger 1 UP");
  gp.rx = 127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Analog Trigger 1 DOWN
  Serial.println("Analog Trigger 1 DOWN");
  gp.rx = -127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Analog Trigger 1 CENTER
  Serial.println("Analog Trigger 1 CENTER");
  gp.rx = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Analog Trigger 2 UP
  Serial.println("Analog Trigger 2 UP");
  gp.ry = 127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Analog Trigger 2 DOWN
  Serial.println("Analog Trigger 2 DOWN");
  gp.ry = -127;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // Analog Trigger 2 CENTER
  Serial.println("Analog Trigger 2 CENTER");
  gp.ry = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Test buttons (up to 32 buttons)
  for (int i = 0; i < 32; ++i) {
    Serial.print("Pressing button ");
    Serial.println(i);
    gp.buttons = (1U << i);
    usb_hid.sendReport(0, &gp, sizeof(gp));
    delay(1000);
  }


  // Random touch
  Serial.println("Random touch");
  gp.x = random(-127, 128);
  gp.y = random(-127, 128);
  gp.z = random(-127, 128);
  gp.rz = random(-127, 128);
  gp.rx = random(-127, 128);
  gp.ry = random(-127, 128);
  gp.hat = random(0, 9);
  gp.buttons = random(0, 0xffff);
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);

  // */
}
