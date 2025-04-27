// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries

#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

/*  If you want to use a specific flash device, for example for a custom built
 board, first look for it in Adafruit_SPIFlash\src\flash_devices.h
 *  If it isn't in there you need to create your own definition like the
 W25Q80DLX_EXAMPLE example below.
 *  These definitions need to be edited to match information on the data sheet
 of the flash device that you want to use.
 *  If you are not sure what the manufacture ID, memory type and capacity values
 should be, try running the sketch anyway and look at the serial output
 *  The flash device will report these values to you as a single hexadecimal
 value (the JDEC ID)
 *  For example, the first device on the list - the W25Q80DLX - will report its
 JDEC ID as 0xef4014, which is made of these three values:
 *  manufacturer_id = 0xef
 *  memory_type     = 0x40
 *  capacity        = 0x14
 *  With this macro properly defined you can then create an array of device
 definitions as shown below, this can include any from the list of devices in
 flash_devices.h, and any you define yourself here
 *  You need to update the variable on line 71 to reflect the number of items in
 the array
 *  You also need to uncomment line 84 and comment out line 81 so this array
 will be passed to the flash memory driver.
 *
 *  Example of a user defined flash memory device:
    #define W25Q80DLX_EXAMPLE                                                \
      {                                                                      \
        .total_size = 1*1024*1024,                                           \
            .start_up_time_us = 5000, .manufacturer_id = 0xef,               \
        .memory_type = 0x40, .capacity = 0x14, .max_clock_speed_mhz = 80,    \
        .quad_enable_bit_mask = 0x02, .has_sector_protection = false,        \
        .supports_fast_read = true, .supports_qspi = true,                   \
        .supports_qspi_writes = false, .write_status_register_split = false, \
        .single_status_byte = false, .is_fram = false,                       \
      }
 */

/*
 * Create an array of data structures and fill it with the settings we defined
 * above. We are using two devices, but more can be added if you want.
 */
// static const SPIFlash_Device_t my_flash_devices[] = {
//     W25Q80DLX_EXAMPLE,
// };
/*
 * Specify the number of different devices that are listed in the array we just
 * created. If you add more devices to the array, update this value to match.
 */
// const int flashDevices = 1;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // wait for native usb
  }

  Serial.println("Adafruit Serial Flash Info example");
  flash.begin();

  // Using a flash device not already listed? Start the flash memory by passing
  // it the array of device settings defined above, and the number of elements
  // in the array.
  // flash.begin(my_flash_devices, flashDevices);

  uint32_t jedec_id = flash.getJEDECID();
  Serial.print("JEDEC ID: 0x");
  Serial.println(jedec_id, HEX);
  Serial.print("Flash size (usable): ");
  Serial.print(flash.size() / 1024);
  Serial.println(" KB");

#ifdef ARDUINO_ARCH_RP2040
  // For rp2 since flash device is also used for storing code, the flash.size()
  // only return usable flash size for data which is dictated by e.g Menu->Flash
  // Size -> 2MB (Sketch 1920KB, FS 64KB) --> size = 64KB For reference purpose,
  // we only try to find and print actual flash device size using JEDEC here
  Serial.print("Flash size (raw): ");
  Serial.print(1 << ((jedec_id & 0xff) - 10));
  Serial.println(" KB");
#endif
}

void loop() {
  // nothing to do
}
