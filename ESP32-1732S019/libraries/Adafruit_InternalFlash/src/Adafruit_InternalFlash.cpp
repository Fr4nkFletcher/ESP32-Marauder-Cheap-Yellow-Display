/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Ha Thach (tinyusb.org) for Adafruit Industries
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

#include "Adafruit_InternalFlash.h"

#define BLOCK_SZ 512

Adafruit_InternalFlash::Adafruit_InternalFlash(uint32_t addr, uint32_t size)
    : _start_addr(addr), _size(size), _flash((const void *)addr, size) {}

bool Adafruit_InternalFlash::begin(void) {
  // nothing to do
  return true;
}

void Adafruit_InternalFlash::end(void) {}

uint32_t Adafruit_InternalFlash::size(void) { return _size; }

uint32_t Adafruit_InternalFlash::sectorCount(void) { return _size / BLOCK_SZ; }

bool Adafruit_InternalFlash::isBusy(void) {
  // always ready since internal flash operation is blocking.
  return true;
}

uint32_t Adafruit_InternalFlash::block2addr(uint32_t block) {
  return _start_addr + block * BLOCK_SZ;
}

//--------------------------------------------------------------------+
// SdFat BaseBlockDRiver API
// A block is 512 bytes
//--------------------------------------------------------------------+
bool Adafruit_InternalFlash::readSector(uint32_t block, uint8_t *dst) {
  return readSectors(block, dst, 1);
}

bool Adafruit_InternalFlash::writeSector(uint32_t block, const uint8_t *src) {
  return writeSectors(block, src, 1);
}

bool Adafruit_InternalFlash::syncDevice() {
  // since block size 512 is larger than 256 byte row size of SAMD21, we don't
  // need any caching
  return true;
}

bool Adafruit_InternalFlash::readSectors(uint32_t block, uint8_t *dst,
                                         size_t nb) {
  uint32_t const addr = block2addr(block);
  memcpy(dst, (void const *)addr, nb * BLOCK_SZ);
  return true;
}

bool Adafruit_InternalFlash::writeSectors(uint32_t block, const uint8_t *src,
                                          size_t nb) {
  // since block size 512 is larger than 256 byte row size of SAMD21, we don't
  // need any caching
  const volatile void *fl_ptr = (const volatile void *)block2addr(block);
  _flash.erase(fl_ptr, nb * BLOCK_SZ);
  _flash.write(fl_ptr, src, nb * BLOCK_SZ);

  return true;
}
