// Adafruit SPI Flash FatFs Full Usage Example
// Author: Tony DiCola
//
// This is an example of all the functions in the SPI Flash
// FatFs library.  Note that you MUST have a flash chip that's
// formatted with a flash filesystem before running.  See the
// fatfs_format example to perform this formatting.
//
// In general the API for this library closely follows the API
// for the Arduino SD card library.  However instead of interacting
// with a global SD object you create an instance of a fatfs class
// and use its open, exists, etc. functions.  See the SD library
// reference for more inspiration and examples to adapt:
//   https://www.arduino.cc/en/reference/SD
//
// Usage:
// - Modify the pins and type of fatfs object in the config
//   section below if necessary (usually not necessary).
// - Upload this sketch to your M0 express board.
// - Open the serial monitor at 115200 baud.  You should see the
//   example start to run and messages printed to the monitor.
//   If you don't see anything close the serial monitor, press
//   the board reset button, wait a few seconds, then open the
//   serial monitor again.
#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

#define D_TEST "/test"
#define D_TEST_FOO_BAR "/test/foo/bar"
#define D_TEST_FOO_BAZ "/test/foo/baz"

#define F_TEST_TEST_TXT "/test/test.txt"
#define F_TEST_FOO_TEST2_TXT "/test/foo/test2.txt"

void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println(F("Adafruit SPI Flash FatFs Full Usage Example"));

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println(F("Error, failed to initialize flash chip!"));
    while (1) {
      yield();
    }
  }
  Serial.print(F("Flash chip JEDEC ID: 0x"));
  Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println(F("Error, failed to mount newly formatted filesystem!"));
    Serial.println(
        F("Was the flash chip formatted with the SdFat_format example?"));
    while (1) {
      yield();
    }
  }
  Serial.println(F("Mounted filesystem!"));

  // Check if a directory called 'test' exists and create it if not there.
  // Note you should _not_ add a trailing slash (like '/test/') to directory
  // names! You can use the same exists function to check for the existence of a
  // file too.
  if (!fatfs.exists(D_TEST)) {
    Serial.println(F("Test directory not found, creating..."));

    // Use mkdir to create directory (note you should _not_ have a trailing
    // slash).
    fatfs.mkdir(D_TEST);

    if (!fatfs.exists(D_TEST)) {
      Serial.println(F("Error, failed to create directory!"));
      while (1) {
        yield();
      }
    } else {
      Serial.println(F("Created directory!"));
    }
  }

  // You can also create all the parent subdirectories automatically with mkdir.
  // For example to create the hierarchy /test/foo/bar:
  Serial.println(F("Creating deep folder structure..."));
  if (!fatfs.exists(D_TEST_FOO_BAR)) {
    Serial.println(F("Creating " D_TEST_FOO_BAR));
    fatfs.mkdir(D_TEST_FOO_BAR);

    if (!fatfs.exists(D_TEST_FOO_BAR)) {
      Serial.println(F("Error, failed to create directory!"));
      while (1) {
        yield();
      }
    } else {
      Serial.println(F("Created directory!"));
    }
  }

  // This will create the hierarchy /test/foo/baz, even when /test/foo already
  // exists:
  if (!fatfs.exists(D_TEST_FOO_BAZ)) {
    Serial.println(F("Creating " D_TEST_FOO_BAZ));
    fatfs.mkdir(D_TEST_FOO_BAZ);

    if (!fatfs.exists(D_TEST_FOO_BAZ)) {
      Serial.println(F("Error, failed to create directory!"));
      while (1) {
        yield();
      }
    } else {
      Serial.println(F("Created directory!"));
    }
  }

  // Create a file in the test directory and write data to it.
  // Note the FILE_WRITE parameter which tells the library you intend to
  // write to the file.  This will create the file if it doesn't exist,
  // otherwise it will open the file and start appending new data to the
  // end of it.
  File32 writeFile = fatfs.open(F_TEST_TEST_TXT, FILE_WRITE);
  if (!writeFile) {
    Serial.println(F("Error, failed to open " F_TEST_TEST_TXT " for writing!"));
    while (1) {
      yield();
    }
  }
  Serial.println(F("Opened file " F_TEST_TEST_TXT " for writing/appending..."));

  // Once open for writing you can print to the file as if you're printing
  // to the serial terminal, the same functions are available.
  writeFile.println("Hello world!");
  writeFile.print("Hello number: ");
  writeFile.println(123, DEC);
  writeFile.print("Hello hex number: 0x");
  writeFile.println(123, HEX);

  // Close the file when finished writing.
  writeFile.close();
  Serial.println(F("Wrote to file " F_TEST_TEST_TXT "!"));

  // Now open the same file but for reading.
  File32 readFile = fatfs.open(F_TEST_TEST_TXT, FILE_READ);
  if (!readFile) {
    Serial.println(F("Error, failed to open " F_TEST_TEST_TXT " for reading!"));
    while (1) {
      yield();
    }
  }

  // Read data using the same read, find, readString, etc. functions as when
  // using the serial class.  See SD library File class for more documentation:
  //   https://www.arduino.cc/en/reference/SD
  // Read a line of data:
  String line = readFile.readStringUntil('\n');
  Serial.print(F("First line of test.txt: "));
  Serial.println(line);

  // You can get the current position, remaining data, and total size of the
  // file:
  Serial.print(F("Total size of test.txt (bytes): "));
  Serial.println(readFile.size(), DEC);
  Serial.print(F("Current position in test.txt: "));
  Serial.println(readFile.position(), DEC);
  Serial.print(F("Available data to read in test.txt: "));
  Serial.println(readFile.available(), DEC);

  // And a few other interesting attributes of a file:
  char readName[64];
  readFile.getName(readName, sizeof(readName));
  Serial.print(F("File name: "));
  Serial.println(readName);
  Serial.print(F("Is file a directory? "));
  Serial.println(readFile.isDirectory() ? F("Yes") : F("No"));

  // You can seek around inside the file relative to the start of the file.
  // For example to skip back to the start (position 0):
  if (!readFile.seek(0)) {
    Serial.println(F("Error, failed to seek back to start of file!"));
    while (1) {
      yield();
    }
  }

  // And finally to read all the data and print it out a character at a time
  // (stopping when end of file is reached):
  Serial.println(F("Entire contents of test.txt:"));
  while (readFile.available()) {
    char c = readFile.read();
    Serial.print(c);
  }

  // Close the file when finished reading.
  readFile.close();

  // You can open a directory to list all the children (files and directories).
  // Just like the SD library the File type represents either a file or
  // directory.
  File32 testDir = fatfs.open(D_TEST);
  if (!testDir) {
    Serial.println(F("Error, failed to open test directory!"));
    while (1) {
      yield();
    }
  }
  if (!testDir.isDirectory()) {
    Serial.println(F("Error, expected test to be a directory!"));
    while (1) {
      yield();
    }
  }
  Serial.println(F("Listing children of directory " D_TEST ":"));
  File32 child = testDir.openNextFile();
  while (child) {
    char filename[64];
    child.getName(filename, sizeof(filename));

    // Print the file name and mention if it's a directory.
    Serial.print(F("- "));
    Serial.print(filename);
    if (child.isDirectory()) {
      Serial.print(F(" (directory)"));
    }
    Serial.println();
    // Keep calling openNextFile to get a new file.
    // When you're done enumerating files an unopened one will
    // be returned (i.e. testing it for true/false like at the
    // top of this while loop will fail).
    child = testDir.openNextFile();
  }

  // If you want to list the files in the directory again call
  // rewindDirectory().  Then openNextFile will start from the
  // top again.
  testDir.rewindDirectory();

  // Delete a file with the remove command.  For example create a test2.txt file
  // inside /test/foo and then delete it.
  File32 test2File = fatfs.open(F_TEST_FOO_TEST2_TXT, FILE_WRITE);
  test2File.close();
  Serial.println(F("Deleting " F_TEST_FOO_TEST2_TXT "..."));
  if (!fatfs.remove(F_TEST_FOO_TEST2_TXT)) {
    Serial.println(F("Error, couldn't delete " F_TEST_FOO_TEST2_TXT " file!"));
    while (1) {
      yield();
    }
  }
  Serial.println(F("Deleted file!"));

  // Delete a directory with the rmdir command.  Be careful as
  // this will delete EVERYTHING in the directory at all levels!
  // I.e. this is like running a recursive delete, rm -rf *, in
  // unix filesystems!
  Serial.println(F("Deleting /test directory and everything inside it..."));
  if (!testDir.rmRfStar()) {
    Serial.println(F("Error, couldn't delete test directory!"));
    while (1) {
      yield();
    }
  }
  // Check that test is really deleted.
  if (fatfs.exists(D_TEST)) {
    Serial.println(F("Error, test directory was not deleted!"));
    while (1) {
      yield();
    }
  }
  Serial.println(F("Test directory was deleted!"));

  Serial.println(F("Finished!"));
}

void loop() {
  // Nothing to be done in the main loop.
  delay(100);
}
