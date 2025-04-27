/*
This example shows how to use PicoDVI and a board's flash filesystem (i.e.
CIRCUITPY drive) simultaneously. This can be useful for loading graphics
files, saving game state in emulators, etc. To keep this simple, it's just
a mash-up of the existing PicoDVI '1bit_text' and Adafruit_CPFS 'simple'
examples, but the same principles certainly apply to other modes.

To start, you first need to temporarily install CircuitPython on the board
to initialize the flash filesystem, which can then be loaded up with the
files you need. https://circuitpython.org/downloads
With the filesystem prepared, one can then work with the Arduino code...

SUPER IMPORTANT: you MUST have current versions of several libraries and
the Earle Philhower arduino-pico core installed. Failure to have all the
right pieces will wipe out any data stored on the drive!

arduino-pico (via Arduino board manager)  3.3.0 or later
Adafruit_CPFS (via Library manager)       1.1.0 or later
Adafruit_SPIFlash (")                     4.2.0 or later

It is wise and STRONGLY RECOMMENDED to keep a backup of any data you install
on the board. These libraries combined are asking a LOT of the RP2040 chip,
and despite best efforts there's still the occasional hiccup that can wipe
the filesystem, making you start over with the CircuitPython install and
drive setup. See notes below about perhaps adding a boot switch to USB-mount
CIRCUITPY only when needed; it's more stable if left unmounted.
*/

#include <PicoDVI.h>       // For DVI video out
#include <Adafruit_CPFS.h> // For accessing the CIRCUITPY drive

FatVolume *fs = NULL; // CIRCUITPY flash filesystem, as a FAT pointer

// This example uses 80x30 monochrome text mode. See other PicoDVI examples
// for color, bitmapped graphics, widescreen, alternate boards, etc.
DVItext1 display(DVI_RES_640x240p60, adafruit_feather_dvi_cfg);

void setup() { // Runs once on startup
  pinMode(LED_BUILTIN, OUTPUT);

  // Start the CIRCUITPY flash filesystem. SUPER IMPORTANT: NOTICE THE
  // EXTRA PARAMETERS HERE. This is REQUIRED when using PicoDVI and
  // Adafruit_CPFS together.
  fs = Adafruit_CPFS::begin(true, -1, NULL, false);
  // The initial 'true' argument tells CPFS to make the flash filesystem
  // available to a USB-connected host computer. Passing 'false' makes it
  // only available to the Arduino sketch. Given the tenuous stability of
  // handling so much at once (DVI, flash, USB), one might want to add a
  // boot-time button or switch to select whether CIRCUITPY is mounted on
  // host, or is just using USB for power.
  // Next two arguments are ignored on RP2040; they're specifically for
  // some 'Haxpress' dev boards with the CPFS library. Last argument should
  // ALWAYS be set 'false' on RP2040, or there will be...trouble.

  if (!display.begin()) { // Start DVI, slow blink LED if insufficient RAM
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }

  if (fs == NULL) { // If CIRCUITPY filesystem is missing or malformed...
    // Show error message & fast blink LED to indicate problem. Full stop.
    display.println("Can't access board's CIRCUITPY drive.");
    display.println("Has CircuitPython been previously installed?");
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 250) & 1);
  } // else valid CIRCUITPY drive, proceed...

  // As in Adafruit_CPFS 'simple' example, allow USB events to settle...
  delay(2500);
  Adafruit_CPFS::change_ack();

  // Then access files and directories using any SdFat calls (open(), etc.)

  // Because fs is a pointer, we use "->" indirection rather than "." access.
  // display pointer is cast to print_t so ls() treats it just like Serial.
  fs->ls((print_t *)&display, LS_R | LS_SIZE); // List initial drive contents
}

void loop() {
  if (Adafruit_CPFS::changed()) { // Anything changed on CIRCUITPY drive?
    Adafruit_CPFS::change_ack();  // Got it, thanks.
    display.println("CIRCUITPY drive contents changed.");
    fs->ls((print_t *)&display, LS_R | LS_SIZE);  // List updated drive contents
  }
}
