/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include "SPI.h"
#include "SdFat.h"
#include "Adafruit_InternalFlash.h"
#include "Adafruit_TinyUSB.h"

// Start address and size should matches value in the CircuitPython (INTERNAL_FLASH_FILESYSTEM = 1)
// to make it easier to switch between Arduino and CircuitPython
#define INTERNAL_FLASH_FILESYSTEM_START_ADDR  (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)
#define INTERNAL_FLASH_FILESYSTEM_SIZE        (64*1024)

// Internal Flash object
Adafruit_InternalFlash flash(INTERNAL_FLASH_FILESYSTEM_START_ADDR, INTERNAL_FLASH_FILESYSTEM_SIZE);

// file system object from SdFat
FatVolume fatfs;

FatFile root;
FatFile file;

// USB MSC object
Adafruit_USBD_MSC usb_msc;

// Set to true when PC write to flash
bool fs_changed;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);

  // Initialize internal flash
  flash.begin();

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "Internal Flash", "1.0");

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_callback, msc_write_callback, msc_flush_callback);
  usb_msc.setWritableCallback(msc_writable_callback);
  usb_msc.setCapacity(flash.size()/512, 512); // Set disk size, block size should be 512 regardless of flash page size
  usb_msc.setUnitReady(true); // Set Lun ready
  usb_msc.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // Init file system on the flash
  fatfs.begin(&flash);

  //while ( !Serial ) delay(10);   // wait for native usb

  fs_changed = true; // to print contents initially
}

void loop() {
  if ( fs_changed ) {
    fs_changed = false;

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
int32_t msc_read_callback (uint32_t lba, void* buffer, uint32_t bufsize) {
  // Note: InternalFlash Block API: readBlocks/writeBlocks/syncBlocks
  // already include sector caching (if needed). We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_callback (uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  // Note: InternalFlash Block API: readBlocks/writeBlocks/syncBlocks
  // already include sector caching (if needed). We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_callback (void) {
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fs_changed = true;
}

// Invoked to check if device is writable as part of SCSI WRITE10
// Default mode is writable
bool msc_writable_callback(void) {
  // true for writable, false for read-only
  return true;
}
