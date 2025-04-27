//
// CYD Resistive Touch Demo
// (cheap yellow display)
//
// This example shows how to use the XPT2046 resistive touch support
// within bb_spi_lcd to read touch info from the LCD. Since the code
// is part of bb_spi_lcd, it automatically scales and rotates the touch
// points to match the display resolution and orientation. Another
// goal of the code is to use the same data structure as the bb_captouch
// library to make it easier to share code between capacitive and resistive
// devices.
//
// Copyright (c) 2024 BitBank Software, Inc.
// written by Larry Bank (bitbank@pobox.com)
// project started March 5, 2024
//
#include <bb_spi_lcd.h>
BB_SPI_LCD lcd;
// These pin definitions are for the most common 2.8" ILI9341 CYD
#define TOUCH_MISO 39
#define TOUCH_MOSI 32
#define TOUCH_CLK 25
#define TOUCH_CS 33

void setup() {
  Serial.begin(115200);
  lcd.begin(DISPLAY_CYD);
  lcd.rtInit(TOUCH_MOSI, TOUCH_MISO, TOUCH_CLK, TOUCH_CS); // initialize touch AFTER initializing the LCD
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setFont(FONT_12x16);
  lcd.println("Touch Test");
  lcd.println("Clear by touching here");
} /* setup() */

void loop() {
TOUCHINFO ti;
  while (1) {
    if (lcd.rtReadTouch(&ti)) {
       if (ti.y[0] < 32) { // user touched the top of the LCD, clear to black
           lcd.fillRect(0, 32, 320, 208, TFT_BLACK);
       } else { // draw a blue filled circle on the touch spot
           lcd.fillRect(ti.x[0], ti.y[0], 4, 4, TFT_BLUE);
       }
    } // if touched
  } // while (1)
} /* loop() */