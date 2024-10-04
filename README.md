# ESP32-Marauder-Cheap-Yellow-Display

<p align="center">
  <img alt="Marauder logo" src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/logo01.png" width="240">
</p>

<p align="center">
  <img src="https://github.com/Fr4nkFletcher/Adafruit_WebSerial_ESPTool/actions/workflows/pages.yml/badge.svg" />
  <img src="https://komarev.com/ghpvc/?username=Fr4nkFletcher&label=Views&color=0e75b6&style=flat" alt="Fr4nkFletcher" />
  <img src="https://img.shields.io/github/issues/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display?style=flat-square" />
</p>

## Project Goal

To port the ESP32-Marauder firmware to the Cheap Yellow Display (CYD) module, enabling advanced WiFi and Bluetooth testing capabilities on an affordable and accessible hardware platform.

## 🏴‍☠️ Latest Update Highlights (09/28/24) 🏴‍☠️

- Flash previous Marauder versions
- [Antenna modification guide](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/AntennaModNew.md) for ESP-WROOM-32U with built-in IPEX/U.FL connector
- [Evil Portal examples and instructions](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/evilportal/)
- [External antenna addition guide](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/AntennaMod.md)

## Requirements for Setup

1. Compatible CYD module (see Compatibility section)
2. Chrome
3. Data-capable USB Cable
4. (Optional) GPS module for enhanced functionality

## Installation Steps

### Web Flasher Method (Recommended)

1. Visit the [CYM-Web-Flasher](https://fr4nkfletcher.github.io/Adafruit_WebSerial_ESPTool/)
2. Click "Connect" and select your device
3. Choose the appropriate Model and Version
4. Click "Program" to start flashing

<p align="center">
  <img src="https://github.com/Fr4nkFletcher/Adafruit_WebSerial_ESPTool/blob/main/assets/sc002.png?raw=true" alt="CYM Web Flasher Screenshot" width="100%" style="max-width:800px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);">
</p>

**Troubleshooting:** If you encounter issues, try:
1. Unplug and restart your CYD
2. Hold RST → tap BOOT → release RST (screen should go blank)
3. Refresh the web flasher and click Connect
4. If problems persist, hold BOOT while clicking Connect

For more details, visit the [Web Flasher repository](https://github.com/Fr4nkFletcher/Adafruit_WebSerial_ESPTool).

### Manual Arduino IDE Method

1. Set up your Arduino environment following the [ESP32 Marauder Arduino IDE Setup Guide](https://github.com/justcallmekoko/ESP32Marauder/wiki/arduino-ide-setup)
2. Add the necessary libraries to your Arduino libraries folder
3. Set the upload speed to `115200` in Arduino IDE (tested with version 1.8.19)
4. Upload the firmware to your CYD module

For a detailed walkthrough, check [Smoochiee's tutorial](https://github.com/smoochiee/MARAUDER-FOR-CYD---CHEAP-YELLOW-DISPLAY).

## Compatibility

Successfully tested on:
- [Module 1](https://amazon.com/dp/B0BVFXR313)
- [Module 2](https://amazon.com/dp/B0CLR7MQ91)

No hardware modifications required, thanks to integration with **@ggaljoen's** [TFT_eSPI](https://github.com/ggaljoen/TFT_eSPI) fork.

## GPS Functionality

GPS is fully operational through the 4-pin connector near the MicroUSB port. For supported GPS hardware, refer to the [official wiki](https://github.com/justcallmekoko/ESP32Marauder/wiki/gps-modification). 

| GPS | -> | CYD |
|-----|:--:|-----|
| VCC | -> | VIN |
| GND | -> | GND |
| TX  | -> | TX  |
| RX  | -> | RX  |

## Example Usage

After flashing, your CYD module will boot into the Marauder interface. Refer to the [ESP32 Marauder Wiki](https://github.com/justcallmekoko/ESP32Marauder/wiki) for detailed usage instructions.

<p align="center">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/2.gif" alt="Demo 1" width="45%">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/swift2.gif" alt="Demo 2" width="45%">
</p>

## Acknowledgments

A big shoutout to the creators and supporters of the [ESP32 Cheap Yellow Display](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display) project and the community Discord, especially **@cod5fgzj**, [**smoochiee**](https://github.com/smoochiee), [**ggaljoen**](https://github.com/ggaljoen), and [**ATOMNFT**](https://github.com/ATOMNFT). And of course [**JustCallMeKoko**](https://github.com/justcallmekoko) for the foundational work on the [ESP32Marauder](https://github.com/justcallmekoko/ESP32Marauder).

## Disclaimer

This project is for educational purposes only. Always obtain proper authorization before testing on networks you don't own or have explicit permission to test.
