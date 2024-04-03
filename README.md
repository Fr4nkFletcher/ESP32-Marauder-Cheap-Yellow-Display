# ESP32-Marauder-Cheap-Yellow-Display

<p align="center">
  <img alt="Marauder logo" src="https://github.com/justcallmekoko/ESP32Marauder/blob/master/pictures/marauder3L.jpg?raw=true" width="300">
</p>
<div align="center">  
  
  ## 🌟 Update Highlight 04/2/24 - [Webserial Terminal](https://fr4nkfletcher.github.io/ESP32-Marauder-Cheap-Yellow-Display/serial_terminal.html) 🌟

</div>

  - **Detect Pwnagotchi [enabled](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/pwn2.jpg) in the WiFi Sniffers submenu (currently only for builds without GPS. Fix hopefully soon)**

- **Version 0.13.9 added to the CYM web flasher, will push files needed to compile from source soon** 

- **Added builds without GPS to the web flasher to allow for serial CLI access to the CYM.**
    
- **Unrelated to the 0.13.9 release: SwiftPair Spam now 100% functional** Samsung, Google, and BLE spam will still crash on occasion but definitely an improvement and usable now. Waiting 10 seconds before each scan seems to help also.

- **For Evil Portal examples, click [here](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/Evil%20Portal/).**

- **Information on adding an external [antenna](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/AntennaMod.md)**


## Web Flasher Instructions
- **Visit:** [CYM-Web-Flasher](https://fr4nkfletcher.github.io/ESP32-Marauder-Cheap-Yellow-Display/flash0.html)

- **Choose your hardware** and click **Connect** to start the flashing process.

## Compatibility

Successfully tested on both of these available on Amazon:
- [Module 1](https://amazon.com/dp/B0BVFXR313)
- [Module 2](https://amazon.com/dp/B0CLR7MQ91)

No hardware modifications required thanks to integration with **@ggaljoen's** [TFT_eSPI](https://github.com/ggaljoen/TFT_eSPI) fork.

- **GPS Functionality**: 🛰 GPS is [enabled](screenshots/gps5.jpg) and fully operational through the 4-pin connector located near the MicroUSB port of the CYD module. [Check here](https://github.com/justcallmekoko/ESP32Marauder/wiki/gps-modification) for details on supported GPS hardware.

## Setup

Add necessary libraries to your Arduino libraries folder. Configure your Arduino environment as detailed in the [ESP32 Marauder Arduino IDE Setup Guide](https://github.com/justcallmekoko/ESP32Marauder/wiki/arduino-ide-setup).

Ensure the upload speed is set to `115200` in Arduino IDE (tested with version 1.8.19).

**If you're having issues or want to do everything yourself, [check Smoochiee's tutorial](https://github.com/smoochiee/MARAUDER-FOR-CYD---CHEAP-YELLOW-DISPLAY) for an in-depth walkthrough of the port.**

## Acknowledgments

A big shoutout to the creators and supporters of the [ESP32 Cheap Yellow Display](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display) project and the community Discord, especially **@cod5fgzj**, [**smoochiee**](https://github.com/smoochiee), and [**ggaljoen**](https://github.com/ggaljoen). And of course JustCallMeKoko for the foundational work on the ESP32 Marauder.

<p align="center">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/2.gif" alt="Demo 1">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/swift2.gif" alt="Demo 2">
</p>

<p align="left"> <img src="https://komarev.com/ghpvc/?username=Fr4nkFletcher&label=Views&color=0e75b6&style=flat" alt="Fr4nkFletcher" />
<img src="https://img.shields.io/github/issues/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display?style=flat-square" />
</p>
