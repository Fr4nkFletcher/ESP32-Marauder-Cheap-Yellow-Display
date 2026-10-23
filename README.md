# Marauder 1.4.3 for CYD 4.0" E32R40T

- Add support for ESP32-4848S040 4.0" ST7796/XPT2046 Resistive Touch (E32R40T)
- Portrait and landscape touch calibration measured on E32R40T hardware
- CYD_40 set as the default active board
- Add `User_Setup_cyd40.h` TFT_eSPI configuration for CYD_40

---

## Requirements

1. A compatible CYD module (see [Compatibility](#compatibility) section below)
2. A data-capable USB cable
3. *(Optional)* GPS module for enhanced functionality — https://github.com/justcallmekoko/ESP32Marauder/wiki/gps-modification

---

## Installation

### Option 1 — esptool.py (Recommended)

Flash directly from the prebuilt binaries in this repo.

Install esptool if you don't have it:

```
pip install esptool
```

Flash:

```
esptool.py --port /dev/YOURSERIALPORT write_flash \
  0x1000  bins/esp32_marauder.ino.bootloader.bin \
  0x8000  bins/esp32_marauder.ino.partitions.bin \
  0x10000 bins/esp32_marauder_v1_4_3_20260627_cyd40_nogps.bin
```

Replace the firmware filename with `cyd40_gps.bin` if you have a GPS module wired up.

#### Troubleshooting:

1. Unplug and restart your CYD module
2. Hold `RST`, tap `BOOT`, release `RST` (the screen should go blank)
3. Re-run the flash command

---

### Option 2 — Arduino IDE (Build from Source)

1. Set up your Arduino environment following the [ESP32 Marauder Arduino IDE Setup Guide](https://github.com/justcallmekoko/ESP32Marauder/wiki/arduino-ide-setup)
2. Update your platform.txt
3. Add the necessary libraries to your Arduino libraries folder
4. Set the upload speed to `115200`
5. In `configs.h`, uncomment the `#define` for your board (default is `CYD_40`)
6. Upload to your CYD module

---

## Compatibility

The following boards are supported. The **4.0" E32R40T** is the default in this fork — for any other board, comment out `#define CYD_40` in `configs.h` and uncomment your board's define.

- **[4.0" Resistive Touch E32R40T](https://www.amazon.com/dp/B0FGJJ24S1)** ← default in this fork
- [1.9" ESP32-S3 No Touch](https://www.aliexpress.us/item/3256807423694742.html)
- [2.4" Capacitive Touch](https://a.co/d/bTSoo9Z)
- [2.4" Resistive Touch](https://a.co/d/fhM7s0J)
- [2.4" Resistive Touch Guition](https://www.aliexpress.us/item/3256806436471011.html)
- [2.8" Resistive MicroUSB-only](https://amazon.com/dp/B0BVFXR313)
- [2.8" Resistive MicroUSB/Type-C 2USB](https://amazon.com/dp/B0CLR7MQ91)
- [3.2" Resistive Touch](https://www.aliexpress.us/item/3256806436888726.html)
- [3.2" Capacitive Touch](https://a.co/d/faB7oVU)
- [3.5" Resistive Touch](https://a.co/d/cxUc73a)
- [3.5" Capacitive Touch](https://a.co/d/2PFDlvL)

No hardware modifications required.

---

## GPS Functionality

GPS is supported via the 4-pin connector near the MicroUSB port. For compatible hardware, refer to the [official wiki](https://github.com/justcallmekoko/ESP32Marauder/wiki/gps-modification).

| GPS | -> | CYD |
|-----|:--:|-----|
| VCC | -> | VIN |
| GND | -> | GND |
| TX  | -> | TX  |
| RX  | -> | RX  |

Note: On 2.4" and 3.5" models swap RX/TX

---

## Example Usage

After flashing, you'll boot into the Marauder interface. Refer to the [ESP32 Marauder Wiki](https://github.com/justcallmekoko/ESP32Marauder/wiki) for detailed usage instructions.

---

## Disclaimer

This project is for educational purposes only. Always obtain proper authorization before testing on networks you don't own or have explicit permission to test.
