/*
 * Print size, modify date/time, and name for all files in root.
 */
#include <SPI.h>

#include "SdFat.h"

#include "Adafruit_SPIFlash.h"

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

File32 root;
File32 file;

//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Init external flash
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
