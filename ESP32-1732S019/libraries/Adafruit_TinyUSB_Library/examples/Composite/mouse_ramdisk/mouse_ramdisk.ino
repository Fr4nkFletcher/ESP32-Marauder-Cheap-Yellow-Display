/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This sketch demonstrates USB Mass Storage and HID mouse (and CDC)
 * - Enumerated as 8KB flash disk
 * - Press button pin will move mouse toward bottom right of monitor
 */

#include "Adafruit_TinyUSB.h"

//--------------------------------------------------------------------+
// MSC RAM Disk Config
//--------------------------------------------------------------------+

// 8KB is the smallest size that windows allow to mount
#define DISK_BLOCK_NUM  16
#define DISK_BLOCK_SIZE 512

#include "ramdisk.h"

Adafruit_USBD_MSC usb_msc;

//--------------------------------------------------------------------+
// HID Config
//--------------------------------------------------------------------+

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object
Adafruit_USBD_HID usb_hid;

#if defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(ARDUINO_NRF52840_CIRCUITPLAY)
const int pin = 4; // Left Button
bool activeState = true;

#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
const int pin = BUTTON_DOWN;
bool activeState = true;

#elif defined PIN_BUTTON1
const int pin = PIN_BUTTON1;
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


// the setup function runs once when you press reset or power the board
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "Mass Storage", "1.0");

  // Set disk size
  usb_msc.setCapacity(DISK_BLOCK_NUM, DISK_BLOCK_SIZE);

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Set Lun ready (RAM disk is always ready)
  usb_msc.setUnitReady(true);

  usb_msc.begin();

  // Set up button
  pinMode(pin, activeState ? INPUT_PULLDOWN : INPUT_PULLUP);

  // Set up HID
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_NONE);
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial.println("Adafruit TinyUSB Mouse + Mass Storage (ramdisk) example");
}

void process_hid() {
  // button is active low
  uint32_t const btn = (digitalRead(pin) == activeState);

  // Remote wakeup
  if (TinyUSBDevice.suspended() && btn) {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }

  /*------------- Mouse -------------*/
  if (usb_hid.ready()) {
    if (btn) {
      int8_t const delta = 5;
      usb_hid.mouseMove(0, delta, delta); // no ID: right + down

      // delay a bit before attempt to send keyboard report
      delay(10);
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

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and 
// return number of copied bytes (must be multiple of block size) 
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  uint8_t const* addr = msc_disk[lba];
  memcpy(buffer, addr, bufsize);

  return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  uint8_t* addr = msc_disk[lba];
  memcpy(addr, buffer, bufsize);

  return bufsize;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb(void) {
  // nothing to do
}
