// Adafruit Internal Flash FatFs Simple Datalogging Example
// Author: Tony DiCola, Ha Thach
//
// This is a simple example that opens a file and prints its
// entire contents to the serial monitor.  Note that
// you MUST have a flash chip that's formatted with a flash
// filesystem before running, and there should be some sort
// of text file on it to open and read.  See the fatfs_format
// example to perform this formatting, and the fatfs_datalogging
// example to write a simple text file.
//
// Usage:
// - Modify the pins and type of fatfs object in the config
//   section below if necessary (usually not necessary).
// - Upload this sketch to your M0 express board.
// - Open the serial monitor at 115200 baud.  You should see the
//   example start to run and messages printed to the monitor.
//   If you don't see anything close the serial monitor, press
//   the board reset buttton, wait a few seconds, then open the
//   serial monitor again.

/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
*********************************************************************/

#include <SPI.h>
#include <SdFat.h>
#include "Adafruit_InternalFlash.h"

// Start address and size should matches value in the CircuitPython (INTERNAL_FLASH_FILESYSTEM = 1)
// to make it easier to switch between Arduino and CircuitPython
#define INTERNAL_FLASH_FILESYSTEM_START_ADDR  (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)
#define INTERNAL_FLASH_FILESYSTEM_SIZE        (64*1024)

Adafruit_InternalFlash flash(INTERNAL_FLASH_FILESYSTEM_START_ADDR, INTERNAL_FLASH_FILESYSTEM_SIZE);

// file system object from SdFat
FatFileSystem fatfs;

// Configuration for the file to open and read:
#define FILE_NAME      "boot_out.txt"

void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Adafruit Internal Flash FatFs Simple File Printing Example");

  // Initialize internal flash
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1) delay(1);
  }

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1) delay(1);
  }
  Serial.println("Mounted filesystem!");

  // Open the file for reading and check that it was successfully opened.
  // The FILE_READ mode will open the file for reading.
  File dataFile = fatfs.open(FILE_NAME, FILE_READ);
  if (dataFile) {
    // File was opened, now print out data character by character until at the
    // end of the file.
    Serial.println("Opened file, printing contents below:");
    while (dataFile.available()) {
      // Use the read function to read the next character.
      // You can alternatively use other functions like readUntil, readString, etc.
      // See the fatfs_full_usage example for more details.
      char c = dataFile.read();
      Serial.print(c);
    }
  }
  else {
    Serial.println("Failed to open data file! Does it exist?");
  }
}

void loop() {
  // Nothing to do in main loop.
  delay(100);
}
