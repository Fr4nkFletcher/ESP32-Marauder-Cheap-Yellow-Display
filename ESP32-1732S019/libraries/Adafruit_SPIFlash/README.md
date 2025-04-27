# Adafruit SPI Flash

[![Build Status](https://github.com/adafruit/Adafruit_SPIFlash/workflows/Build/badge.svg)](https://github.com/adafruit/Adafruit_SPIFlash/actions) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

Arduino library for external (Q)SPI flash device.

## Supported Cores

- [adafruit/Adafruit_nRF52_Arduino](https://github.com/adafruit/Adafruit_nRF52_Arduino)
- [adafruit/ArduinoCore-samd](https://github.com/adafruit/ArduinoCore-samd)
- [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico)
- [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32)

## Features

- Support SPI interfaces for all cores
- Support QSPI interfaces for nRF52 and SAMD51
- Support FRAM flash devices
- Provide raw flash access APIs
- Implement block device APIs from SdFat's BaseBlockDRiver with caching to facilitate FAT filesystem on flash device
