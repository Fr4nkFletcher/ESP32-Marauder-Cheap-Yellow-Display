/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach for Adafruit
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

#include "tusb_option.h"

#if defined ARDUINO_ARCH_RP2040 && CFG_TUD_ENABLED

#include "Arduino.h"

// mbed old pico-sdk need to wrap with cpp
extern "C" {
#include "hardware/flash.h"
#include "hardware/irq.h"
#include "pico/bootrom.h"
#include "pico/mutex.h"
#include "pico/time.h"
}

#include "arduino/Adafruit_TinyUSB_API.h"
#include "tusb.h"

// SDK >= 1.4.0 need to dynamically request the IRQ to avoid conflicts
#if (PICO_SDK_VERSION_MAJOR * 100 + PICO_SDK_VERSION_MINOR) < 104
#define USB_TASK_IRQ 31
#else
static unsigned int USB_TASK_IRQ;
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
// rp2040 implementation will install appropriate handler when initializing
// tinyusb. There is no need to forward IRQ from application
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Earle Philhower and mbed specific
//--------------------------------------------------------------------+

// mbed use old pico-sdk does not have unique_id
#ifdef ARDUINO_ARCH_MBED

#define get_unique_id(_serial) flash_get_unique_id(_serial)

#else

#include "pico/unique_id.h"
#define get_unique_id(_serial)                                                 \
  pico_get_unique_board_id((pico_unique_board_id_t *)_serial);

#endif

//--------------------------------------------------------------------+
// Porting API
//--------------------------------------------------------------------+

// Big, global USB mutex, shared with all USB devices to make sure we don't
// have multiple cores updating the TUSB state in parallel
mutex_t __usb_mutex;

static void usb_task_irq(void) {
  // if the mutex is already owned, then we are in user code
  // in this file which will do a tud_task itself, so we'll just do nothing
  // until the next tick; we won't starve
  if (mutex_try_enter(&__usb_mutex, NULL)) {
    tud_task();
    mutex_exit(&__usb_mutex);
  }
}

#ifndef PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY
#define PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY 0x00
#endif

// invoked when there is hardware usb irq, trigger task runner later
static void usb_task_trigger_irq(void) { irq_set_pending(USB_TASK_IRQ); }

void TinyUSB_Port_InitDevice(uint8_t rhport) {
  mutex_init(&__usb_mutex);

  tud_init(rhport);

  // soft irq for task runner
#if (PICO_SDK_VERSION_MAJOR * 100 + PICO_SDK_VERSION_MINOR) >= 104
  USB_TASK_IRQ = user_irq_claim_unused(true);
#endif
  irq_set_exclusive_handler(USB_TASK_IRQ, usb_task_irq);
  irq_set_enabled(USB_TASK_IRQ, true);

  // add shared irq to trigger task runner
  irq_add_shared_handler(USBCTRL_IRQ, usb_task_trigger_irq,
                         PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY);
}

void TinyUSB_Port_EnterDFU(void) {
  reset_usb_boot(0, 0);
  while (1) {
  }
}

uint8_t TinyUSB_Port_GetSerialNumber(uint8_t serial_id[16]) {
  get_unique_id(serial_id);
  return FLASH_UNIQUE_ID_SIZE_BYTES;
}

//--------------------------------------------------------------------+
// Core API
// Implement Core API since rp2040 need mutex for calling tud_task in
// IRQ context
//--------------------------------------------------------------------+

extern "C" {

void TinyUSB_Device_Task(void) {
  // Since tud_task() is also invoked in ISR, we need to get the mutex first
  if (mutex_try_enter(&__usb_mutex, NULL)) {
    tud_task();
    mutex_exit(&__usb_mutex);
  }
}
}

#endif
