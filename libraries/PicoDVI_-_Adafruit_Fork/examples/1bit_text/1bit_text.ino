// 1-bit (black, white) text mode for PicoDVI.

#include <PicoDVI.h>

// Here's how an 80x30 character display is declared. First argument,
// resolution, is full display pixel count...character cells are 8x8 pixels,
// yielding the 80x30 result. 640x240 uses "tall" pixels, the result of all
// this is very reminiscent of IBM VGA mode. Second argument is a hardware
// configuration -- examples are written for Adafruit Feather RP2040 DVI,
// but that's easily switched out for boards like the Pimoroni Pico DV
// (use 'pimoroni_demo_hdmi_cfg') or Pico DVI Sock ('pico_sock_cfg').
DVItext1 display(DVI_RES_640x240p60, adafruit_feather_dvi_cfg);

// Wider and taller modes are possible. Wider pushes overclocking even
// higher than 640x480 mode. SOME BOARDS MIGHT SIMPLY NOT BE COMPATIBLE
// WITH THIS. May require selecting QSPI div4 clock (Tools menu) to slow
// down flash accesses, may require over-volting the CPU to 1.25 or 1.3 V.
// Here's how a 100x60 char display might be declared:
//DVItext1 display(DVI_RES_800x480p60, adafruit_feather_dvi_cfg);

// A reduced refresh rate display doesn't as aggressive an over-clock
// This timing is verified to work on https://www.adafruit.com/product/2232
//DVItext1 display(DVI_RES_800x240p30, adafruit_feather_dvi_cfg);

void setup() { // Runs once on startup
  if (!display.begin()) { // Blink LED if insufficient RAM
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }
}

void loop() {
  display.print("Hello World!  ");
  delay(50);
}
