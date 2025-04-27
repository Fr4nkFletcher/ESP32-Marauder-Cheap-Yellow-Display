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

/* Example sketch read analog pin (default A0) and log it to LOG_FILE on the msc device
 * every LOG_INTERVAL ms. */

// nRF52 and ESP32 use freeRTOS, we may need to run USBhost.task() in its own rtos's thread.
// Since USBHost.task() will put loop() into dormant state and prevent followed code from running
// until there is USB host event.
#if defined(ARDUINO_NRF52_ADAFRUIT) || defined(ARDUINO_ARCH_ESP32)
  #define USE_FREERTOS
#endif

// SdFat is required for using Adafruit_USBH_MSC_SdFatDevice
#include "SdFat.h"

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"

#define LOG_FILE        "cpu_temp.csv"
#define LOG_INTERVAL    5000

// Analog pin for reading
const int analogPin = A0;

// USB Host MSC Block Device object which implemented API for use with SdFat
Adafruit_USBH_MSC_BlockDevice msc_block_dev;

// file system object from SdFat
FatVolume fatfs;
File32 f_log;

// if file system is successfully mounted on usb block device
volatile bool is_mounted = false;

void data_log(void) {
  if (!is_mounted) {
    // nothing to do
    return;
  }

  static unsigned long last_ms = 0;
  unsigned long ms = millis();

  if ( ms - last_ms < LOG_INTERVAL ) {
    return;
  }

  // Turn on LED when start writing
  digitalWrite(LED_BUILTIN, HIGH);

  f_log = fatfs.open(LOG_FILE, O_WRITE | O_APPEND | O_CREAT);

  if (!f_log) {
    Serial.println("Cannot create file: " LOG_FILE);
  } else {
    int value = analogRead(analogPin);

    Serial.printf("%lu,%d\r\n", ms, value);
    f_log.printf("%lu,%d\r\n", ms, value);

    f_log.close();
  }

  last_ms = ms;
  Serial.flush();
}

#ifdef USE_FREERTOS

#ifdef ARDUINO_ARCH_ESP32
  #define USBH_STACK_SZ 2048
#else
  #define USBH_STACK_SZ 200
#endif

void usbhost_rtos_task(void *param) {
  (void) param;
  while (1) {
    USBHost.task();
  }
}

#endif

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  // init host stack on controller (rhport) 1
  // For rp2040: this is called in core1's setup1()
  USBHost.begin(1);
#endif

#ifdef USE_FREERTOS
  // Create a task to run USBHost.task() in background
  xTaskCreate(usbhost_rtos_task, "usbh", USBH_STACK_SZ, NULL, 3, NULL);
#endif

//  while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("TinyUSB Host MassStorage Data Logger Example");
}

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
//--------------------------------------------------------------------+
// Using Host shield MAX3421E controller
//--------------------------------------------------------------------+
void loop() {
#ifndef USE_FREERTOS
  USBHost.task();
#endif
  data_log();
}

#elif defined(ARDUINO_ARCH_RP2040)
//--------------------------------------------------------------------+
// For RP2040 use both core0 for device stack, core1 for host stack
//--------------------------------------------------------------------+
void loop() {
  data_log();
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
bool write_complete_callback(uint8_t dev_addr, tuh_msc_complete_data_t const *cb_data) {
  (void) dev_addr;
  (void) cb_data;

  // turn off LED after write is complete
  // Note this only marks the usb transfer is complete, device can take longer to actual
  // write data to physical flash
  digitalWrite(LED_BUILTIN, LOW);

  return true;
}

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
  // Initialize block device with MSC device address
  msc_block_dev.begin(dev_addr);

  // For simplicity this example only support LUN 0
  msc_block_dev.setActiveLUN(0);
  msc_block_dev.setWriteCompleteCallback(write_complete_callback);
  is_mounted = fatfs.begin(&msc_block_dev);

  if (is_mounted) {
    fatfs.ls(&Serial, LS_SIZE);
  } else {
    Serial.println("Failed to mount mass storage device. Make sure it is formatted as FAT");
  }
}

// Invoked when a device with MassStorage interface is unmounted
void tuh_msc_umount_cb(uint8_t dev_addr) {
  (void) dev_addr;

  // unmount file system
  is_mounted = false;
  fatfs.end();

  // end block device
  msc_block_dev.end();
}

}
