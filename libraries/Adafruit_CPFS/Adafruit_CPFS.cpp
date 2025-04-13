// SPDX-FileCopyrightText: 2023 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

/*!
 * @file Adafruit_CPFS.cpp
 *
 * This is a barebones library to:
 *
 * - Make a CircuitPython-capable board's flash filesystem accessible to
 *   Arduino code.
 * - Make this same drive accessible to a host computer over USB.
 *
 * This is an "80/20" library to cover the most common use case, with least
 * code and documentation, for non-technical users: if a board supports
 * CircuitPython, then Arduino code and a host computer can both access that
 * drive. Flash formatting is done by installing CircuitPython once
 * (pre-built for just about everything), no special steps. That's it.
 * NOT for SD cards, special flash partitioning, etc. Those can always be
 * implemented manually using the Adafruit_TinyUSB library, but this is
 * not the code for it. Keeping it really simple.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "Paint Your Dragon" Burgess for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#if defined(USE_TINYUSB) || defined(ESP32)

#include "Adafruit_CPFS.h"
#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>
#if defined(_SAMD21_)
#include <Adafruit_InternalFlash.h>
// These apply to M0 boards only, ignored elsewhere:
#define INTERNAL_FLASH_FS_SIZE (64 * 1024)
#define INTERNAL_FLASH_FS_START (0x00040000 - 256 - 0 - INTERNAL_FLASH_FS_SIZE)
#endif

// Library state is maintained in a few global variables (rather than in the
// Adafruit_CPFS class) as there's only one filesystem instance anyway, and
// also that the mass storage callbacks are C and require access to this info.

// For several major board types, the correct flash transport to use
// is known at compile-time:
#if defined(ARDUINO_ARCH_ESP32)
static Adafruit_FlashTransport_ESP32 _transport;
#elif defined(ARDUINO_ARCH_RP2040)
static Adafruit_FlashTransport_RP2040_CPY *_transport;
#elif defined(EXTERNAL_FLASH_USE_QSPI)
static Adafruit_FlashTransport_QSPI _transport;
#elif defined(EXTERNAL_FLASH_USE_CS) && defined(EXTERNAL_FLASH_USE_SPI)
static Adafruit_FlashTransport_SPI _transport(EXTERNAL_FLASH_USE_CS,
                                              EXTERNAL_FLASH_USE_SPI);
#else
// If not one of the above board types, nor EXTERNAL_FLASH_USE_* defined,
// it's probably a SAMD21. Some can be "Haxpress" modified to add external
// SPI flash & run a special CircuitPython build, but Arduino IDE lacks a
// distinct special board select...at compile-time, indistinguishable from
// a stock M0 board, could go either way. Thus, the transport and flash
// members are pointers, initialized at run-time depending on arguments
// passed (or not) to the begin() function.
#define HAXPRESS
static Adafruit_FlashTransport_SPI *_transport; // Unused if internal
static void *_flash;                            // Is cast internal/SPI as needed in callbacks
#endif
#if !defined HAXPRESS
static Adafruit_SPIFlash *_flash;
#endif

static Adafruit_USBD_MSC _usb_msc;
static FatVolume _fatfs;
static bool _started = 0;
static bool _changed = 0;

#if defined(HAXPRESS)

// On Haxpress-capable boards, flash type (internal vs SPI) isn't known
// at compile time, so callbacks are provided for both, and one set or
// other is installed in begin().

static int32_t msc_read_cb_internal(uint32_t lba, void *buffer,
                                    uint32_t bufsize) {
  return ((Adafruit_InternalFlash *)_flash)
                 ->readBlocks(lba, (uint8_t *)buffer, bufsize / 512)
             ? bufsize
             : -1;
}

static int32_t msc_write_cb_internal(uint32_t lba, uint8_t *buffer,
                                     uint32_t bufsize) {
  _changed = 1;
  return ((Adafruit_InternalFlash *)_flash)
                 ->writeBlocks(lba, buffer, bufsize / 512)
             ? bufsize
             : -1;
}

static void msc_flush_cb_internal(void) {
  ((Adafruit_InternalFlash *)_flash)->syncBlocks();
  _fatfs.cacheClear();
}

static int32_t msc_read_cb_spi(uint32_t lba, void *buffer, uint32_t bufsize) {
  return ((Adafruit_SPIFlash *)_flash)
                 ->readBlocks(lba, (uint8_t *)buffer, bufsize / 512)
             ? bufsize
             : -1;
}

static int32_t msc_write_cb_spi(uint32_t lba, uint8_t *buffer,
                                uint32_t bufsize) {
  _changed = 1;
  return ((Adafruit_SPIFlash *)_flash)->writeBlocks(lba, buffer, bufsize / 512)
             ? bufsize
             : -1;
}

static void msc_flush_cb_spi(void) {
  ((Adafruit_SPIFlash *)_flash)->syncBlocks();
  _fatfs.cacheClear();
}

#else

// Flash type is known at compile time. Simple callbacks.

static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  return _flash->readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize
                                                                   : -1;
}

static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  _changed = 1;
  return _flash->writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

static void msc_flush_cb(void) {
  _flash->syncBlocks();
  _fatfs.cacheClear();
}

#endif // end !HAXPRESS

FatVolume *Adafruit_CPFS::begin(bool msc, int cs, void *spi, bool idle) {

  if (_started)
    return &_fatfs; // Don't re-init if already running

  _started = 1;

#if defined(HAXPRESS)

  if ((cs >= 0) && (spi != NULL)) { // External flash
    if ((_transport = new Adafruit_FlashTransport_SPI(cs, (SPIClass *)spi))) {
      if ((_flash = (void *)new Adafruit_SPIFlash(_transport))) {
        ((Adafruit_SPIFlash *)_flash)->begin();
        if (msc) {
          _usb_msc.setID("Adafruit", "External Flash", "1.0");
          _usb_msc.setReadWriteCallback(msc_read_cb_spi, msc_write_cb_spi,
                                        msc_flush_cb_spi);
          _usb_msc.setCapacity(((Adafruit_SPIFlash *)_flash)->size() / 512,
                               512);
          _usb_msc.setUnitReady(true);
          _usb_msc.begin();
        }
        if (_fatfs.begin((Adafruit_SPIFlash *)_flash))
          return &_fatfs;
      }    // end if new flash
    }      // end if new transport
  } else { // Internal flash
    if ((_flash = (void *)new Adafruit_InternalFlash(INTERNAL_FLASH_FS_START,
                                                     INTERNAL_FLASH_FS_SIZE))) {
      ((Adafruit_InternalFlash *)_flash)->begin();
      if (msc) {
        _usb_msc.setID("Adafruit", "Internal Flash", "1.0");
        _usb_msc.setReadWriteCallback(
            msc_read_cb_internal, msc_write_cb_internal, msc_flush_cb_internal);
        _usb_msc.setCapacity(((Adafruit_InternalFlash *)_flash)->size() / 512,
                             512);
        _usb_msc.setUnitReady(true);
        _usb_msc.begin();
      }
      if (_fatfs.begin((Adafruit_InternalFlash *)_flash))
        return &_fatfs;
    } // end if new flash
  }

#else

#if defined(ARDUINO_ARCH_RP2040)
  if ((_transport = new Adafruit_FlashTransport_RP2040_CPY(idle))) {
    if ((_flash = new Adafruit_SPIFlash(_transport))) {
#else
  { // _transport is declared globally, no test needed, pass address-of
    if ((_flash = new Adafruit_SPIFlash(&_transport))) {
#endif

      _flash->begin();
      if (msc) {
        _usb_msc.setID("Adafruit", "Onboard Flash", "1.0");
        _usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
        _usb_msc.setCapacity(_flash->size() / 512, 512);
        _usb_msc.setUnitReady(true);
        _usb_msc.begin();
      }

      if (_fatfs.begin(_flash))
        return &_fatfs;
    } // end if new flash
  }   // end if new transport

#endif // end HAXPRESS

  _started = 0;
  return NULL;
}

bool Adafruit_CPFS::changed(void) { return _changed; }
void Adafruit_CPFS::change_ack(void) { _changed = 0; }

#endif // end USE_TINYUSB || ESP32
