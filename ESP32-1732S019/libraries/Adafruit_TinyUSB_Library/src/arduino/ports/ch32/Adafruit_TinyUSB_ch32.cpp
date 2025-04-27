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

#if CFG_TUD_ENABLED &&                                                         \
    (defined(ARDUINO_ARCH_CH32) || defined(CH32V20x) || defined(CH32V30x))

#include "Arduino.h"
#include "arduino/Adafruit_USBD_Device.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
extern "C" {

// USBD (fsdev)
#if CFG_TUD_WCH_USBIP_FSDEV
__attribute__((interrupt("WCH-Interrupt-fast"))) void
USB_LP_CAN1_RX0_IRQHandler(void) {
  tud_int_handler(0);
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void
USB_HP_CAN1_TX_IRQHandler(void) {
  tud_int_handler(0);
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void
USBWakeUp_IRQHandler(void) {
  tud_int_handler(0);
}
#endif

// USBFS
#if CFG_TUD_WCH_USBIP_USBFS

#if defined(CH32V10x) || defined(CH32V20x)

#if defined(CH32V10x)
#define USBHDWakeUp_IRQHandler USBWakeUp_IRQHandler
#endif

__attribute__((interrupt("WCH-Interrupt-fast"))) void USBHD_IRQHandler(void) {
  tud_int_handler(0);
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void
USBHDWakeUp_IRQHandler(void) {
  tud_int_handler(0);
}
#endif

#ifdef CH32V30x
__attribute__((interrupt("WCH-Interrupt-fast"))) void OTG_FS_IRQHandler(void) {
  tud_int_handler(0);
}
#endif

#endif

// USBHS
#if CFG_TUD_WCH_USBIP_USBHS
__attribute__((interrupt("WCH-Interrupt-fast"))) void USBHS_IRQHandler(void) {
  tud_int_handler(0);
}
#endif

void yield(void) {
  tud_task();
  if (tud_cdc_connected()) {
    tud_cdc_write_flush();
  }
}
}

//--------------------------------------------------------------------+
// Porting API
//--------------------------------------------------------------------+

void TinyUSB_Port_InitDevice(uint8_t rhport) {
#if CFG_TUD_WCH_USBIP_FSDEV || CFG_TUD_WCH_USBIP_USBFS
  // Full speed OTG or FSDev

#if defined(CH32V10x)
  EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
  EXTEN->EXTEN_CTR &= ~EXTEN_USB_5V_SEL;

#define RCC_AHBPeriph_OTG_FS RCC_AHBPeriph_USBHD
#endif

  uint8_t usb_div;
  switch (SystemCoreClock) {
#if defined(CH32V20x) || defined(CH32V30x)
  case 48000000:
    usb_div = 0; // div1
    break;
  case 96000000:
    usb_div = 1; // div2
    break;
  case 144000000:
    usb_div = 2; // div3
    break;
#elif defined(CH32V10x)
  case 48000000:
    usb_div = RCC_USBCLKSource_PLLCLK_Div1;
    break;
  case 72000000:
    usb_div = RCC_USBCLKSource_PLLCLK_1Div5;
    break;
#endif
  default:
    return; // unsupported
  }

#if defined(CH32V30x)
  RCC_OTGFSCLKConfig(usb_div);
#else
  RCC_USBCLKConfig(usb_div);
#endif

#if CFG_TUD_WCH_USBIP_FSDEV
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif

#if CFG_TUD_WCH_USBIP_USBFS
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE);
#endif
#endif

#if CFG_TUD_WCH_USBIP_USBHS
  // High speed USB: currently require 144MHz HSE, update later
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
#endif

  tud_init(rhport);
}

void TinyUSB_Port_EnterDFU(void) {
  // Reset to Boot ROM
  // Serial1.println("Reset to Boot ROM");
  //__disable_irq();
  // tud_deinit(0);
  //
  // // define function pointer to BOOT ROM address
  // void (*bootloader_entry)(void) = (void (*)(void))0x1FFF8000;
  // bootloader_entry();
  // while(1) {}
}

uint8_t TinyUSB_Port_GetSerialNumber(uint8_t serial_id[16]) {
  volatile uint32_t *ch32_uuid = ((volatile uint32_t *)0x1FFFF7E8UL);
  uint32_t *serial_32 = (uint32_t *)serial_id;
  serial_32[0] = ch32_uuid[0];
  serial_32[1] = ch32_uuid[1];
  serial_32[2] = ch32_uuid[2];

  return 12;
}

#endif
