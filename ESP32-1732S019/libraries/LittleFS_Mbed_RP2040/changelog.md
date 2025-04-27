# LittleFS_Mbed_RP2040 Library

[![arduino-library-badge](https://www.ardu-badge.com/badge/LittleFS_Mbed_RP2040.svg?)](https://www.ardu-badge.com/LittleFS_Mbed_RP2040)
[![GitHub release](https://img.shields.io/github/release/khoih-prog/LittleFS_Mbed_RP2040.svg)](https://github.com/khoih-prog/LittleFS_Mbed_RP2040/releases)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/khoih-prog/LittleFS_Mbed_RP2040/blob/main/LICENSE)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](#Contributing)
[![GitHub issues](https://img.shields.io/github/issues/khoih-prog/LittleFS_Mbed_RP2040.svg)](http://github.com/khoih-prog/LittleFS_Mbed_RP2040/issues)

---
---

## Table of Contents

* [Changelog](#changelog)
  * [Releases v1.1.0](#releases-v110)
  * [Releases v1.0.3](#releases-v103)
  * [Releases v1.0.2](#releases-v102)
  * [Releases v1.0.1](#releases-v101)
  * [Initial Releases v1.0.0](#initial-releases-v100)

---
---

## Changelog

### Releases v1.1.0

1. Fix `multiple-definitions` linker error. Check [Different behaviour using the src_cpp or src_h lib #80](https://github.com/khoih-prog/ESPAsync_WiFiManager/discussions/80)
2. Update all examples

### Releases v1.0.3

1. Fix crashing issue for new flash. Check [RP2040_RTC_Time crashes Pico, does not work #3](https://github.com/khoih-prog/RP2040_RTC/issues/3)

### Releases v1.0.1

1. Fix FORCE_REFORMAT bug in example
2. Change default RP2040_FS_SIZE_KB from 256KB to 64KB to avoid crash in some new boards. Check [MBED crash - RP2040 rebooting #1](https://github.com/khoih-prog/LittleFS_Mbed_RP2040/issues/1)

### Initial Releases v1.0.0

1. Initial coding to support RP2040-based boards such as **Nano_RP2040_Connect, RASPBERRY_PI_PICO**, etc. using [**Arduino-mbed RP2040** core](https://github.com/arduino/ArduinoCore-mbed)



