# Adafruit_CPFS

[C]ircuit[P]ython [F]ile[S]ystem library for Arduino.

This is a small library that does one...well okay, two...things:

* Makes a CircuitPython-capable board's flash filesystem accessible to
  Arduino code.
* Make this same drive accessible to a host computer over USB.

Nothing new here, same can be done using Adafruit_TinyUSB and SdFat, this
simply wraps the code in a library out of sight, as it's normally quite a
tangle of #ifdefs. NOT for SD cards or unusual flash partitioning. Again,
those can be implemented with the aforementioned libraries. This is an
"80/20" library to cover the most common use case, with least code and
documentation, for non-technical users: if a board supports CircuitPython,
then Arduino code and a host computer can both access that drive. Flash
formatting is done by installing CircuitPython once (pre-built for just
about everything), no special steps. That's it.

When an Arduino sketch using this library is uploaded to the board,
CircuitPython will be overwritten, but the flash drive contents remain
intact and are available to our code. CircuitPython can be reinstalled
later via bootloader, drive again remains intact. Best of both worlds!

Must enable TinyUSB before compiling:
Tools -> USB Stack -> Adafruit_TinyUSB

IMPORTANT: keep a backup of the CIRCUITPY drive contents somewhere safe.
USB + mass storage + Serial is pretty demanding, and accidents do happen.
You may need to reinstall CircuitPython to initialize a botched drive.

See examples/simple for use.
