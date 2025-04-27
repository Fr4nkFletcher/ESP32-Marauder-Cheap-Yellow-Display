// "Logo bounce" example for PicoDVI library. If just starting out,
// see the 1bit_double_buffer which explains the PicoDVI groundwork.
// Comments in THIS file are mostly distinct & new concepts.

// IF NO OUTPUT OR RED FLICKER SCANLINES: try Tools->Optimize->(-O3)

#include <PicoDVI.h>
#include "sprite.h" // Graphics data

DVIGFX1 display(DVI_RES_640x480p60, true, adafruit_feather_dvi_cfg);

// See notes in 1bit_double_buffer regarding 800x480 mode.
//DVIGFX1 display(DVI_RES_800x480p60, true, adafruit_feather_dvi_cfg);
// May also require -O3 setting.

int x  = 0; // Start logo at
int y  = 0; // top left corner,
int vx = 1; // moving right
int vy = 1; // and down

void setup() { // Runs once on startup
  if (!display.begin()) { // Blink LED if insufficient RAM
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }
}

void loop() { // Runs once every frame
  display.fillScreen(0); // Clear back framebuffer, then draw sprite:
  display.drawBitmap(x, y, sprite, SPRITE_WIDTH, SPRITE_HEIGHT, 1);

  // Swap front/back buffers, do not duplicate current screen state
  // to next frame, we'll draw it new from scratch each time.
  display.swap();

  // Update sprite position, bouncing off all 4 sides
  x += vx; // Horizontal
  if ((x == 0) || (x == (display.width() - SPRITE_WIDTH))) vx *= -1;
  y += vy; // Vertical
  if ((y == 0) || (y == (display.height() - SPRITE_HEIGHT))) vy *= -1;
}
