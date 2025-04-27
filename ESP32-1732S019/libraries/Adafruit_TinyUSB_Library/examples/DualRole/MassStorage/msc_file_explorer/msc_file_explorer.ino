/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example demonstrates use of both device and host, where
 * - Device run on native usb controller (roothub port0)
 * - Host depending on MCUs run on either:
 *   - rp2040: bit-banging 2 GPIOs with the help of Pico-PIO-USB library (roothub port1)
 *   - samd21/51, nrf52840, esp32: using MAX3421e controller (host shield)
 *
 * Requirements:
 * - For rp2040:
 *   - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 *   - 2 consecutive GPIOs: D+ is defined by PIN_USB_HOST_DP, D- = D+ +1
 *   - Provide VBus (5v) and GND for peripheral
 *   - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 * - For samd21/51, nrf52840, esp32:
 *   - Additional MAX2341e USB Host shield or featherwing is required
 *   - SPI instance, CS pin, INT pin are correctly configured in usbh_helper.h
 */

// SdFat is required for using Adafruit_USBH_MSC_SdFatDevice
#include "SdFat.h"

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"

// USB Host MSC Block Device object which implemented API for use with SdFat
Adafruit_USBH_MSC_BlockDevice msc_block_dev;

// file system object from SdFat
FatVolume fatfs;

// if file system is successfully mounted on usb block device
bool is_mounted = false;

void setup() {
  Serial.begin(115200);

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  // init host stack on controller (rhport) 1
  // For rp2040: this is called in core1's setup1()
  USBHost.begin(1);
#endif

  // while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("TinyUSB Host Mass Storage File Explorer Example");
}


#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
//--------------------------------------------------------------------+
// Using Host shield MAX3421E controller
//--------------------------------------------------------------------+
void loop() {
  USBHost.task();
  Serial.flush();
}

#elif defined(ARDUINO_ARCH_RP2040)
//--------------------------------------------------------------------+
// For RP2040 use both core0 for device stack, core1 for host stack
//--------------------------------------------------------------------+
void loop() {
}

//------------- Core1 -------------//
void setup1() {
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

void loop1() {
  USBHost.task();
}

#endif

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
extern "C"
{

// Invoked when device is mounted (configured)
void tuh_mount_cb(uint8_t daddr) {
  (void) daddr;
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr) {
  (void) daddr;
}

// Invoked when a device with MassStorage interface is mounted
void tuh_msc_mount_cb(uint8_t dev_addr) {
  Serial.printf("Device attached, address = %d\r\n", dev_addr);

  // Initialize block device with MSC device address
  msc_block_dev.begin(dev_addr);

  // For simplicity this example only support LUN 0
  msc_block_dev.setActiveLUN(0);

  is_mounted = fatfs.begin(&msc_block_dev);

  if (is_mounted) {
    fatfs.ls(&Serial, LS_SIZE);
  }
}

// Invoked when a device with MassStorage interface is unmounted
void tuh_msc_umount_cb(uint8_t dev_addr) {
  Serial.printf("Device removed, address = %d\r\n", dev_addr);

  // unmount file system
  is_mounted = false;
  fatfs.end();

  // end block device
  msc_block_dev.end();
}

}