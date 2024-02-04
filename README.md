# ESP32 Marauder with Cheap Yellow Display

![Marauder logo](https://github.com/justcallmekoko/ESP32Marauder/blob/master/pictures/marauder3L.jpg?raw=true)

The ESP32 Marauder project is now compatible with affordable yellow displays, making it more accessible without needing hardware modifications. This guide provides instructions for quick setup and configuration.

## Quick Start with Web Flasher

1. Go to [ESPTool Web Flasher](http://espressif.github.io/esptool-js).
2. Set baud rate to `115200` and flash address to `0x10000`.
3. Choose `esp32_marauder_v0_13_5_20240203_cyd.bin` (or `_inverted.bin` for color issues).

## Compatibility

Compatible with specific ESP32 modules listed on Amazon ([Link 1](https://amazon.com/dp/B0BVFXR313), [Link 2](https://amazon.com/dp/B0CLR7MQ91)).

## Setup and Configuration

- Add required libraries to your Arduino libraries folder.
- Configure Arduino IDE according to the [ESP32 Marauder Guide](https://github.com/justcallmekoko/ESP32Marauder/wiki/arduino-ide-setup).

## Acknowledgments

Thanks to [TFT_eSPI](https://github.com/ggaljoen/TFT_eSPI), [ESP32 Cheap Yellow Display](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display), and the community Discord for their support, especially @cod5fgzj. And of course JuatCallMeKoko for making this a thing.

![Project Screenshot](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/2.gif)