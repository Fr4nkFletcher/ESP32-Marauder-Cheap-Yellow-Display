/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example demo how to expose on-board external Flash as USB Mass Storage.
 * Following library is required
 *   - Adafruit_SPIFlash https://github.com/adafruit/Adafruit_SPIFlash
 *   - SdFat https://github.com/adafruit/SdFat
 *
 * Note: Adafruit fork of SdFat enabled ENABLE_EXTENDED_TRANSFER_CLASS and FAT12_SUPPORT
 * in SdFatConfig.h, which is needed to run SdFat on external flash. You can use original
 * SdFat library and manually change those macros
 *
 * Note2: If your flash is not formatted as FAT12 previously, you could format it using
 * follow sketch https://github.com/adafruit/Adafruit_SPIFlash/tree/master/examples/SdFat_format
 */

#include "SPI.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

FatFile root;
FatFile file;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Check if flash is formatted
bool fs_formatted = false;

// Set to true when PC write to flash
bool fs_changed = true;;

// the setup function runs once when you press reset or power the board
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  flash.begin();

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size()/512, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  usb_msc.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Init file system on the flash
  fs_formatted = fatfs.begin(&flash);

  //while ( !Serial ) delay(10);   // wait for native usb

  Serial.println("Adafruit TinyUSB Mass Storage External Flash example");
  Serial.print("JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.print(flash.size() / 1024); Serial.println(" KB");
}

void loop() {
  // check if formatted
  if ( !fs_formatted ) {
    fs_formatted = fatfs.begin(&flash);

    if (!fs_formatted) {
      Serial.println("Failed to init files system, flash may not be formatted");
      Serial.println("Please format it as FAT12 with your PC or using Adafruit_SPIFlash's SdFat_format example:");
      Serial.println("- https://github.com/adafruit/Adafruit_SPIFlash/tree/master/examples/SdFat_format");
      Serial.println();

      delay(1000);
      return;
    }
  }

  if ( fs_changed ) {
    fs_changed = false;

    Serial.println("Opening root");

    if ( !root.open("/") ) {
      Serial.println("open root failed");
      return;
    }

    Serial.println("Flash contents:");

    // Open next file in root.
    // Warning, openNext starts at the current directory position
    // so a rewind of the directory may be required.
    while ( file.openNext(&root, O_RDONLY) ) {
      file.printFileSize(&Serial);
      Serial.write(' ');
      file.printName(&Serial);
      if ( file.isDir() ) {
        // Indicate a directory.
        Serial.write('/');
      }
      Serial.println();
      file.close();
    }

    root.close();

    Serial.println();
    delay(1000); // refresh every 1 second
  }
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and 
// return number of copied bytes (must be multiple of block size) 
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize) {
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  digitalWrite(LED_BUILTIN, HIGH);

  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void) {
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fs_changed = true;

  digitalWrite(LED_BUILTIN, LOW);
}
