/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach and Dean Miller for Adafruit Industries LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Adafruit_SPIFlash.h"

#ifdef USE_TINYUSB
#include "Adafruit_TinyUSB.h"
#endif

#define LOGICAL_BLOCK_SIZE 512

#if SPIFLASH_DEBUG
#define SPIFLASH_LOG(_block, _count)                                           \
  do {                                                                         \
    Serial.print(__FUNCTION__);                                                \
    Serial.print(": lba = ");                                                  \
    Serial.print(_block);                                                      \
    if (_count) {                                                              \
      Serial.print(" count = ");                                               \
      Serial.print(_count);                                                    \
    }                                                                          \
    Serial.println();                                                          \
  } while (0)
#else
#define SPIFLASH_LOG(_sector, _count)
#endif

Adafruit_SPIFlash::Adafruit_SPIFlash() : Adafruit_SPIFlashBase() {
  _cache_en = true;
  _cache = NULL;
}

Adafruit_SPIFlash::Adafruit_SPIFlash(Adafruit_FlashTransport *transport,
                                     bool useCache)
    : Adafruit_SPIFlashBase(transport) {
  _cache_en = useCache;
  _cache = NULL;
}

bool Adafruit_SPIFlash::begin(SPIFlash_Device_t const *flash_devs,
                              size_t count) {
  bool ret = Adafruit_SPIFlashBase::begin(flash_devs, count);

#ifndef __AVR__
  // Use cache if not FRAM
  // Note: Skip caching if AVR. Comment out since new cache on AVR seems to
  // corrupt memory rather than safely return NULL
  if (_flash_dev && !_flash_dev->is_fram) {
    if (_cache_en && !_cache) {
      _cache = new Adafruit_FlashCache;
    }
  }
#endif

  return ret;
}

void Adafruit_SPIFlash::end(void) {
  // invoke base class end
  Adafruit_SPIFlashBase::end();

  if (_cache != NULL) {
    delete _cache;
    _cache = NULL;
  }
}

//--------------------------------------------------------------------+
// SdFat BaseBlockDRiver API
// A block is 512 bytes
//--------------------------------------------------------------------+

bool Adafruit_SPIFlash::isBusy() { return !Adafruit_SPIFlashBase::isReady(); }

uint32_t Adafruit_SPIFlash::sectorCount() {
  return Adafruit_SPIFlashBase::size() / LOGICAL_BLOCK_SIZE;
}

bool Adafruit_SPIFlash::readSector(uint32_t block, uint8_t *dst) {
  SPIFLASH_LOG(block, 1);

  if (_cache) {
    return _cache->read(this, block * LOGICAL_BLOCK_SIZE, dst,
                        LOGICAL_BLOCK_SIZE);
  } else {
    // FRAM does not need caching
    return this->readBuffer(block * LOGICAL_BLOCK_SIZE, dst,
                            LOGICAL_BLOCK_SIZE) > 0;
  }
}

bool Adafruit_SPIFlash::syncDevice() {
  SPIFLASH_LOG(0, 0);

  if (_cache) {
    return _cache->sync(this);
  } else {
    return true;
  }
}

bool Adafruit_SPIFlash::writeSector(uint32_t block, const uint8_t *src) {
  SPIFLASH_LOG(block, 1);

  if (_cache) {
    return _cache->write(this, block * LOGICAL_BLOCK_SIZE, src,
                         LOGICAL_BLOCK_SIZE);
  } else {
    return this->writeBuffer(block * LOGICAL_BLOCK_SIZE, src,
                             LOGICAL_BLOCK_SIZE) > 0;
  }
}

bool Adafruit_SPIFlash::readSectors(uint32_t block, uint8_t *dst, size_t nb) {
  SPIFLASH_LOG(block, nb);

  if (_cache) {
    return _cache->read(this, block * LOGICAL_BLOCK_SIZE, dst,
                        LOGICAL_BLOCK_SIZE * nb);
  } else {
    return this->readBuffer(block * LOGICAL_BLOCK_SIZE, dst,
                            LOGICAL_BLOCK_SIZE * nb) > 0;
  }
}

bool Adafruit_SPIFlash::writeSectors(uint32_t block, const uint8_t *src,
                                     size_t nb) {
  SPIFLASH_LOG(block, nb);
  if (_cache) {
    return _cache->write(this, block * LOGICAL_BLOCK_SIZE, src,
                         LOGICAL_BLOCK_SIZE * nb);
  } else {
    return this->writeBuffer(block * LOGICAL_BLOCK_SIZE, src,
                             LOGICAL_BLOCK_SIZE * nb) > 0;
  }
}
