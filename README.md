# ESP32-Marauder-Cheap-Yellow-Display

<p align="center">
  <img alt="Marauder logo" src="https://github.com/justcallmekoko/ESP32Marauder/blob/master/pictures/marauder3L.jpg?raw=true" width="300">
</p>

## Web Flasher Instructions

- **Visit:** [ESPTool Web Flasher](http://espressif.github.io/esptool-js)
- **Settings:**
  - Baud Rate: `115200`
  - Flash Address: `0x10000`
- **Firmware:** Select `esp32_marauder_v0_13_5_20240203_cyd.bin`. Use `_inverted.bin` if the display colors are inverted.
- **Action:** Click **Program** to start the flashing process.

## Compatibility

Successfully tested on ESP32 modules available on Amazon:
- [Module 1](https://amazon.com/dp/B0BVFXR313)
- [Module 2](https://amazon.com/dp/B0CLR7MQ91)

No hardware modifications required thanks to integration with [TFT_eSPI](https://github.com/ggaljoen/TFT_eSPI).

## Setup

Add necessary libraries to your Arduino libraries folder. Configure your Arduino environment as detailed in the [ESP32 Marauder Arduino IDE Setup Guide](https://github.com/justcallmekoko/ESP32Marauder/wiki/arduino-ide-setup).

Ensure the upload speed is set to `115200` in Arduino IDE (tested with version 1.8.19).

## Acknowledgments

A big shoutout to the creators and supporters of the [ESP32 Cheap Yellow Display](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display) project and the community Discord, especially @cod5fgzj. Kudos to JustCallMeKoko for the foundational work on the ESP32 Marauder.

<p align="center">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/2.gif" alt="Demo 1">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/1.gif" alt="Demo 2">
</p>