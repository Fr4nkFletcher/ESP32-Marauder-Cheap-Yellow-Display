// "Boing" ball example for PicoDVI library, If just starting out,
// see the 8bit_double_buffer which explains the PicoDVI groundwork.
// Comments in THIS file are mostly distinct & new concepts.
// Adapted from PyPortal example in Adafruit_ILI9341 library.

#include <PicoDVI.h>
#include "graphics.h" // Graphics data

// 4:3 aspect for that Amiga flavour:
DVIGFX8 display(DVI_RES_320x240p60, true, adafruit_feather_dvi_cfg);

#define YBOTTOM   123  // Ball Y coord at bottom
#define YBOUNCE  -3.5  // Upward velocity on ball bounce

// Ball coordinates are stored floating-point because screen refresh
// is so quick, whole-pixel movements are just too fast!
float ballx     = 20.0, bally     = YBOTTOM, // Current ball position
      ballvx    =  0.8, ballvy    = YBOUNCE, // Ball velocity
      ballframe = 3;                         // Ball animation frame #
int   balloldx  = ballx, balloldy = bally;   // Prior ball position

void setup() {
  if (!display.begin()) { // Blink LED if insufficient RAM
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }

  // Set up color palette
  display.setColor(0, 0xAD75); // #0 = Background color
  display.setColor(1, 0xA815); // #1 = Grid color
  display.setColor(2, 0x5285); // #2 = Background in shadow
  display.setColor(3, 0x600C); // #3 = Grid in shadow

  // Draw initial framebuffer contents (grid, no shadow):
  display.drawBitmap(0, 0, (uint8_t *)background, 320, 240, 1, 0);
  display.swap(true, true); // Duplicate same bg & palette into both buffers
}

void loop() {
  balloldx = (int16_t)ballx; // Save prior position
  balloldy = (int16_t)bally;
  ballx   += ballvx;         // Update position
  bally   += ballvy;
  ballvy  += 0.06;           // Update Y velocity
  if((ballx <= 15) || (ballx >= display.width() - BALLWIDTH))
    ballvx *= -1;            // Left/right bounce
  if(bally >= YBOTTOM) {     // Hit ground?
    bally  = YBOTTOM;        // Clip and
    ballvy = YBOUNCE;        // bounce up
  }

  // Determine screen area to update.  This is the bounds of the ball's
  // prior and current positions, so the old ball is fully erased and new
  // ball is fully drawn.
  int16_t minx, miny, maxx, maxy, width, height;
  // Determine bounds of prior and new positions
  minx = ballx;
  if(balloldx < minx)                    minx = balloldx;
  miny = bally;
  if(balloldy < miny)                    miny = balloldy;
  maxx = ballx + BALLWIDTH  - 1;
  if((balloldx + BALLWIDTH  - 1) > maxx) maxx = balloldx + BALLWIDTH  - 1;
  maxy = bally + BALLHEIGHT - 1;
  if((balloldy + BALLHEIGHT - 1) > maxy) maxy = balloldy + BALLHEIGHT - 1;

  width  = maxx - minx + 1;
  height = maxy - miny + 1;

  // Ball animation frame # is incremented opposite the ball's X velocity
  ballframe -= ballvx * 0.5;
  if(ballframe < 0)        ballframe += 14; // Constrain from 0 to 13
  else if(ballframe >= 14) ballframe -= 14;

  // Set 7 palette entries to white, 7 to red, based on frame number.
  // This makes the ball spin.
  for(uint8_t i=0; i<14; i++) {
    display.setColor(i + 4, ((((int)ballframe + i) % 14) < 7) ? 0xFFFF : 0xF800);
  }

  // Only the changed rectangle is drawn into the 'renderbuf' array...
  uint8_t  c, *destPtr;
  int16_t  bx  = minx - (int)ballx, // X relative to ball bitmap (can be negative)
           by  = miny - (int)bally, // Y relative to ball bitmap (can be negative)
           bgx = minx,              // X relative to background bitmap (>= 0)
           bgy = miny,              // Y relative to background bitmap (>= 0)
           x, y, bx1, bgx1;         // Loop counters and working vars
  uint8_t  p;                       // 'packed' value of 2 ball pixels
  int8_t bufIdx = 0;

  uint8_t *buf = display.getBuffer(); // -> back buffer

  for(y=0; y<height; y++) { // For each row...
    destPtr = &buf[display.width() * (miny + y) + minx];
    bx1  = bx;  // Need to keep the original bx and bgx values,
    bgx1 = bgx; // so copies of them are made here (and changed in loop below)
    for(x=0; x<width; x++) {
      if((bx1 >= 0) && (bx1 < BALLWIDTH) &&  // Is current pixel row/column
         (by  >= 0) && (by  < BALLHEIGHT)) { // inside the ball bitmap area?
        // Yes, do ball compositing math...
        p = ball[by][bx1 / 2];                // Get packed value (2 pixels)
        c = (bx1 & 1) ? (p & 0xF) : (p >> 4); // Unpack high or low nybble
        if(c == 0) { // Outside ball - just draw grid
          c = background[bgy][bgx1 / 8] & (0x80 >> (bgx1 & 7)) ? 1 : 0;
        } else if(c > 1) { // In ball area...
          c += 2; // Convert to color index >= 4
        } else { // In shadow area, draw shaded grid...
          c = background[bgy][bgx1 / 8] & (0x80 >> (bgx1 & 7)) ? 3 : 2;
        }
      } else { // Outside ball bitmap, just draw background bitmap...
        c = background[bgy][bgx1 / 8] & (0x80 >> (bgx1 & 7)) ? 1 : 0;
      }
      *destPtr++ = c; // Store pixel color
      bx1++;  // Increment bitmap position counters (X axis)
      bgx1++;
    }
    by++; // Increment bitmap position counters (Y axis)
    bgy++;
  }

  display.swap(true, false); // Show & copy current background buffer to next
}
