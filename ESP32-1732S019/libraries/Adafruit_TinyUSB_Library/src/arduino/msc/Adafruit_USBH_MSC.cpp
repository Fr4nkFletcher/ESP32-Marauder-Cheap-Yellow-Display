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

// ESP32 out-of-sync
#ifdef ARDUINO_ARCH_ESP32
#include "arduino/ports/esp32/tusb_config_esp32.h"
#endif

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUH_MSC

#include "Adafruit_USBH_MSC.h"
#include "tusb.h"

#if __has_include("SdFat.h")

Adafruit_USBH_MSC_BlockDevice::Adafruit_USBH_MSC_BlockDevice() {
  _daddr = _lun = 0;
  _busy = false;
  _wr_cb = NULL;
}

bool Adafruit_USBH_MSC_BlockDevice::begin(uint8_t dev_addr) {
  _daddr = dev_addr;
  return true;
}

bool Adafruit_USBH_MSC_BlockDevice::setActiveLUN(uint8_t lun) {
  _lun = lun;
  return true;
}
void Adafruit_USBH_MSC_BlockDevice::setWriteCompleteCallback(
    tuh_msc_complete_cb_t cb) {
  _wr_cb = cb;
}

void Adafruit_USBH_MSC_BlockDevice::end(void) { _daddr = _lun = 0; }

bool Adafruit_USBH_MSC_BlockDevice::mounted(void) { return _daddr > 0; }

bool Adafruit_USBH_MSC_BlockDevice::isBusy(void) { return _busy; }

bool Adafruit_USBH_MSC_BlockDevice::wait_for_io(void) {
  while (_busy) {
    if (tuh_task_event_ready()) {
      tuh_task();
    }
  }

  return true;
}

bool Adafruit_USBH_MSC_BlockDevice::_io_complete_cb(
    uint8_t dev_addr, tuh_msc_complete_data_t const *cb_data) {
  (void)cb_data;
  if (dev_addr != _daddr) {
    // something wrong occurred, maybe device removed while transferring
    return false;
  }

  // TODO skip csw status: assuming io is successful
  _busy = false;

  switch (cb_data->cbw->command[0]) {
  case SCSI_CMD_WRITE_10:
    if (_wr_cb) {
      _wr_cb(dev_addr, cb_data);
    }
    break;
  }

  return true;
}

static bool _msc_io_complete_cb(uint8_t dev_addr,
                                tuh_msc_complete_data_t const *cb_data) {
  Adafruit_USBH_MSC_BlockDevice *sdfat_dev =
      (Adafruit_USBH_MSC_BlockDevice *)cb_data->user_arg;
  sdfat_dev->_io_complete_cb(dev_addr, cb_data);
  return true;
}

uint32_t Adafruit_USBH_MSC_BlockDevice::sectorCount(void) {
  return tuh_msc_get_block_count(_daddr, _lun);
}

bool Adafruit_USBH_MSC_BlockDevice::syncDevice(void) {
  // no caching
  return true;
}

bool Adafruit_USBH_MSC_BlockDevice::readSectors(uint32_t block, uint8_t *dst,
                                                size_t ns) {
  _busy = true;
  if (tuh_msc_read10(_daddr, _lun, dst, block, (uint16_t)ns,
                     _msc_io_complete_cb, (uintptr_t)this)) {
    wait_for_io();
    return true;
  } else {
    _busy = false;
    return false;
  }
}

bool Adafruit_USBH_MSC_BlockDevice::writeSectors(uint32_t block,
                                                 const uint8_t *src,
                                                 size_t ns) {
  _busy = true;
  if (tuh_msc_write10(_daddr, _lun, src, block, (uint16_t)ns,
                      _msc_io_complete_cb, (uintptr_t)this)) {
    wait_for_io();
    return true;
  } else {
    _busy = false;
    return false;
  }
}

bool Adafruit_USBH_MSC_BlockDevice::readSector(uint32_t block, uint8_t *dst) {
  return readSectors(block, dst, 1);
}

bool Adafruit_USBH_MSC_BlockDevice::writeSector(uint32_t block,
                                                const uint8_t *src) {
  return writeSectors(block, src, 1);
}

#endif

#endif
