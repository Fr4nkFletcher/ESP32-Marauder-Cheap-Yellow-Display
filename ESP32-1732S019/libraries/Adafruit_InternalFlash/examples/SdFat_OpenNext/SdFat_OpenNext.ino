/*
 * Print size, modify date/time, and name for all files in root.
 */

/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
*********************************************************************/

#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_InternalFlash.h"

// Start address and size should matches value in the CircuitPython (INTERNAL_FLASH_FILESYSTEM = 1)
// to make it easier to switch between Arduino and CircuitPython
#define INTERNAL_FLASH_FILESYSTEM_START_ADDR  (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)
#define INTERNAL_FLASH_FILESYSTEM_SIZE        (64*1024)

Adafruit_InternalFlash flash(INTERNAL_FLASH_FILESYSTEM_START_ADDR, INTERNAL_FLASH_FILESYSTEM_SIZE);

// file system object from SdFat
FatFileSystem fatfs;

FatFile root;
FatFile file;

//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Init internal flash
  flash.begin();

  // Init file system on the flash
  fatfs.begin(&flash);
  
  // Wait for USB Serial 
  while (!Serial) {
    yield();
  }
  
  if (!root.open("/")) {
    Serial.println("open root failed");
  }
  // Open next file in root.
  // Warning, openNext starts at the current directory position
  // so a rewind of the directory may be required.
  while (file.openNext(&root, O_RDONLY)) {
    file.printFileSize(&Serial);
    Serial.write(' ');
    file.printModifyDateTime(&Serial);
    Serial.write(' ');
    file.printName(&Serial);
    if (file.isDir()) {
      // Indicate a directory.
      Serial.write('/');
    }
    Serial.println();
    file.close();
  }
  
  if (root.getError()) {
    Serial.println("openNext failed");
  } else {
    Serial.println("Done!");
  }
}
//------------------------------------------------------------------------------
void loop() {}
