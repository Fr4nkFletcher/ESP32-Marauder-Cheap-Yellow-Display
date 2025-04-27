/*
  Internal Flash read/write

 This example shows how to read and write data to and from internal flash
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13

 created   Nov 2010 by David A. Mellis
 modified 9 Apr 2012 by Tom Igoe
 modified 13 Jan 2021 by Ha Thach

 This example code is in the public domain.

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

File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing Filesystem on internal flash...");
  
  // Init internal flash
  flash.begin();

//  static const uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
//  uint32_t page_size = pageSizes[NVMCTRL->PARAM.bit.PSZ];
//  Serial.printf("Page size: %u, Row size: %u\r\n", page_size, page_size*4);

  // Open file system on the flash
  if ( !fatfs.begin(&flash) ) {
    Serial.println("Error: filesystem is not existed. Please try SdFat_format example to make one.");
    while(1) yield();
  }

  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = fatfs.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = fatfs.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}
