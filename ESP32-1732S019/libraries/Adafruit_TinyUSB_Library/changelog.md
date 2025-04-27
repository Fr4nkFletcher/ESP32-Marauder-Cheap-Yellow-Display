# Adafruit TinyUSB Arduino Library Changelog

## 1.6.0

- Update TinyUSB to post 0.12.0 at commit https://github.com/hathach/tinyusb/commit/b4a0f0b273eee32ead7acbd44ca9554c58a2adfa
- Fix build with ESP32S2 v2.0.1rc
- Fix MIDI Control Change message sending is corrupted

## 1.5.0 - 2021.09.29

- Add support for ESP32-S2 core version 2.0.0
  - ESP32 port relies on Espressif's [esp32-hal-tinyusb.c](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-tinyusb.c) for building usb descriptors which requires all descriptors must be specified in usb objects declaration i.e constructors. Therefore all descriptor-related fields must be part of object declaration and descriptor-related API have no effect afterwards for this port.

- Add new constructor for Adafruit_USBD_HID(desc_report, len, protocol, interval_ms, has_out_endpoint)

## 1.4.4 - 2021.08.18

- Update tinyusb stack
- Fully support nRF5x suspend, resume & remote wakeup
- Update hid_keyboard/mouse example to hid boot keyboard/mouse

## 1.4.3 - 2021.08.11

- Fix HID generic inout example python/js script cause missing 1st byte in report
- Use correct HID code for newline 0x28

## 1.4.2 - 2021.08.03

- Update tinyusb to match upstream

## 1.4.1 - 2021.07.27

- Fix nRF race condition when initialize usb stack

## 1.4.0 - 2021.07.22

- Support rp2040 mbed core as non built-in support

## 1.3.2 - 2021.07.07

- revert CFG_TUSB_DEBUG = 0 on SAMD port

## 1.3.1 - 2021.07.06

- Fix warning with ci build for rp2040 core
- Fix example build for esp32s2
- Use ARDUINO_NRF52_ADAFRUIT instead of ARDUINO_ARCH_NRF52

## 1.3.0 - 2021.06.29

- Move tusb_config for each ports into library to make it more portable
- Adding support for ESP32S2 (still need PR to be merged from esp32-arduino)
- Update CDC to have instance and getInstanceCount()
- Allow USB_VID/PID to be declared in variant pins_arduino.h
- Use bug report form template

## 1.1.0 - 2021.06.21

- Add support for multiple CDC ports (need to modify tusb_config.h)
- fix #86 when calling midi API when device is not fully enumerated
- fix Serial connection issue with nrf52 on windows
- Update device driver of rp2040 to match tinyusb upstream
- Added feather rp2040 to ci build (skipped the external flash example for now due to SdFat compilation error)
- Add optional debug log for stack with Serial1 on samd core (will expand later to other core).
- lost more bug fixes to the stack

## 1.0.3 - 2021.05.26

- Update tinyusb stack to latest
- Added HID Gamepad example with Dhat support
- Fix warnings with -Wall -Wextra
- Fix warnings with cast function type for nrf

## 1.0.1 - 2021.05.21

Warn user to select TinyUSB via menu on ports where it is optional e.g SAMD and RP2040 

## 1.0.0 - 2021.05.19

Rework to work as independent libraries, sources from https://github.com/adafruit/Adafruit_TinyUSB_ArduinoCore is now integrated as part of this libraries. Require 
- Adafruit SAMD core with version at least 1.7.0
- Adafruit nRF52 core with version at least 0.22.0
- Support [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico) version released after https://github.com/earlephilhower/arduino-pico/pull/127

## 0.9.0 - 2020.04.23

- Fixed mouseButtonRelease() error
- Supported multiple cables for USB MIDI (requires BSP nRF52 0.20.0 and SAMD 1.5.12 )
- Added consumer control support to HID/hid_composite example
- Added Adafruit_USBD_HID send report helper: sendReport8(), sendReport16(), sendReport32()

**Minor Breaking Changes**
- Removed trailing comma in hid report descriptor, this is required to use with BSP nRF52 0.20.0 and SAMD 1.5.12 e.g

from 

```
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_KEYBOARD( HID_REPORT_ID(RID_KEYBOARD), ),
  TUD_HID_REPORT_DESC_MOUSE   ( HID_REPORT_ID(RID_MOUSE   ), ),
};
```
to 

```
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_KEYBOARD( HID_REPORT_ID(RID_KEYBOARD) /*, no more trailing comma */ ),
  TUD_HID_REPORT_DESC_MOUSE   ( HID_REPORT_ID(RID_MOUSE   )  /*, no more trailing comma */ ),
};
```

## 0.8.2 - 2020.04.06

- Removed package-lock.json in hid generic inout example due to security warning from github

## 0.8.1 - 2020.01.08

- More CI migrating work, no function changes

## 0.8.0 - 2019.12.30

- Correct USB BCD version to 2.1 for webUSB
- Migrate CI from travis to github actions

## 0.7.1 - 2019.10.12

- Fixed MIDI build failed since it is under development

## 0.7.0 - 2019.10.09

- Added MIDI virtual wires/plugs

## 0.6.0 - 2019.08.05

- Added webUSB support with 2 example: webusb-serial, webusb-rgb
- Aligned mouse examples, added newer hid in/out example from main repo, added new composite example for ramdisk and hid in/out. PR #19 thanks to @PTS93

## 0.5.0 - 2019.07.17

- Added travis build
- Fixed msc setID
- Added itfnum to internal API getDescriptor() 
- Added MIDI support
  - Added midi_test example
  - Added pizza box dj example for cplayground express
- Mass Storage
  - Added msc_sdfat, msc dual lun (external flash + sd card)  example  
  - Updated msc example to use new SPIFlash 3.x API
  - Update msc example to print root contents
- HID
  - Added hid_mouse, hid_keyboard
  - Added hid_composite_joy_featherwing
- Added Composite: mouse_ramdisk, mouse_external_flash example

## 0.0.1 Initial Release

