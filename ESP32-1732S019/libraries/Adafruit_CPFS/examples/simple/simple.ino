/*
Barebones example for Adafruit_CPFS. Lists CIRCUITPY drive contents to
the Serial console. If anything on the drive changes, a new listing is
shown.

Board must have CircuitPython installed at least once to initialize
the flash filesystem. When this Arduino sketch is uploaded to board,
CircuitPython will be overwritten, but the flash drive contents remain
intact and are available to our code. CircuitPython can be reinstalled
later via bootloader, drive again remains intact. Best of both worlds!

Must enable TinyUSB before compiling:
Tools -> USB Stack -> Adafruit_TinyUSB

IMPORTANT: keep a backup of the CIRCUITPY drive contents somewhere safe.
USB + mass storage + Serial is pretty demanding, and accidents do happen.
*/

#include <Adafruit_CPFS.h>

FatVolume *fs = NULL; // CIRCUITPY flash filesystem, as a FAT pointer

void setup(void) {
  // Start the CIRCUITPY flash filesystem FIRST. Very important!
  fs = Adafruit_CPFS::begin();
  // An optional initial true/false flag passed to begin() selects whether
  // the flash filesystem is presented to a USB-attached host computer (true),
  // or accessible only to code on the microcontroller (false).
  // For "Haxpress" boards (small M0 boards retrofitted with SPI flash),
  // a chip-select pin and/or SPI instance can follow like so:
  // fs = Adafruit_CPFS::begin(true, SS1, &SPI1); // QT Py M0 Haxpress

  // Start Serial AFTER Adafruit_CPFS, or CIRCUITPY won't show on computer.
  Serial.begin(115200);
  //while(!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

  if (fs == NULL) { // If CIRCUITPY filesystem is missing or malformed...
    // Show error message & blink LED to indicate problem. Full stop.
    Serial.println("Can't access board's CIRCUITPY drive.");
    Serial.println("Has CircuitPython been previously installed?");
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  } // else valid CIRCUITPY drive, proceed...

  // Most simple programs can jump right in at this point, such as reading
  // settings or graphics files. Because this particular example monitors
  // for changes in the filesystem, it's good to pause here -- a LOT happens
  // in begin() when the filesystem is connected to USB, and many rapid-fire
  // change events occur. Allow a couple seconds to settle...
  delay(2500);
  Adafruit_CPFS::change_ack(); // Clear any pent-up change notifications

  // Then access files and directories using any SdFat calls (open(), etc.)

  // Because fs is a pointer, we use "->" indirection rather than "." access.
  fs->ls("/", LS_R | LS_SIZE); // List initial drive contents
}

void loop(void) {
  if (Adafruit_CPFS::changed()) { // Anything changed on CIRCUITPY drive?
    Adafruit_CPFS::change_ack();  // Got it, thanks.
    Serial.println("CIRCUITPY drive contents changed.");
    fs->ls("/", LS_R | LS_SIZE);  // List updated drive contents
  }
  // Note that "changes" are often inconsequential -- updating the last-
  // touch times when clicking a file from the host computer, for example.
  // You might see the directory listing refresh multiple times even when
  // nothing of much substance has occurred on the drive. This is normal.
  // Most projects need not even concern themselves with change detection.
}
