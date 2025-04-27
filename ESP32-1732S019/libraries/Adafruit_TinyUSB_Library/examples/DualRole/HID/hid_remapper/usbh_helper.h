/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#ifndef USBH_HELPER_H
#define USBH_HELPER_H

#ifdef ARDUINO_ARCH_RP2040
  // pio-usb is required for rp2040 host
  #include "pio_usb.h"

  // Pin D+ for host, D- = D+ + 1
  #ifndef PIN_USB_HOST_DP
  #define PIN_USB_HOST_DP  16
  #endif

  // Pin for enabling Host VBUS. comment out if not used
  #ifndef PIN_5V_EN
  #define PIN_5V_EN        18
  #endif

  #ifndef PIN_5V_EN_STATE
  #define PIN_5V_EN_STATE  1
  #endif
#endif // ARDUINO_ARCH_RP2040

#include "Adafruit_TinyUSB.h"

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  // USB Host using MAX3421E: SPI, CS, INT
  #include "SPI.h"

  #if defined(ARDUINO_METRO_ESP32S2)
    Adafruit_USBH_Host USBHost(&SPI, 15, 14);
  #elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    Adafruit_USBH_Host USBHost(&SPI, 33, 15);
  #else
    // Default CS and INT are pin 10, 9
    Adafruit_USBH_Host USBHost(&SPI, 10, 9);
  #endif
#else
  // Native USB Host such as rp2040
  Adafruit_USBH_Host USBHost;
#endif

//--------------------------------------------------------------------+
// Helper Functions
//--------------------------------------------------------------------+

#ifdef ARDUINO_ARCH_RP2040
static void rp2040_configure_pio_usb(void) {
  //while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
    while (!Serial) {
      delay(10);   // wait for native usb
    }
    Serial.printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n");
    while (1) {
      delay(1);
    }
  }

#ifdef PIN_5V_EN
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, PIN_5V_EN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  // For pico-w, PIO is also used to communicate with cyw43
  // Therefore we need to alternate the pio-usb configuration
  // details https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
#endif

  USBHost.configure_pio_usb(1, &pio_cfg);
}
#endif

#endif
