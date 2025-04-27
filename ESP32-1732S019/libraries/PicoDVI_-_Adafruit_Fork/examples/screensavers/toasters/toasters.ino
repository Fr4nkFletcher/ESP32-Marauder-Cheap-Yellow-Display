// "Flying toasters" example for PicoDVI library. If just starting out,
// see the 8bit_double_buffer which explains the PicoDVI groundwork.
// Comments in THIS file are mostly distinct & new concepts.

// IF NO OUTPUT OR RED FLICKER SCANLINES: try Tools->Optimize->(-O3)

#include <PicoDVI.h>
#include "sprites.h" // Graphics data

DVIGFX8 display(DVI_RES_320x240p60, true, adafruit_feather_dvi_cfg);

// See notes in 8bit_double_buffer regarding 400x240 mode.
//DVIGFX8 display(DVI_RES_400x240p60, true, adafruit_feather_dvi_cfg);
// Also requires -O3 setting.

// This structure holds pointers to sprite graphics and masks in sprites.h.
// First 8 are toasters, last 3 are toast.
const struct {
  const uint8_t *sprite;
  const uint8_t *mask;
} spritedata[] = {
  { toaster0, toaster0_mask}, // There are really only 4 unique frames
  { toaster1, toaster1_mask}, // of toaster flapping. Instead of code to
  { toaster2, toaster2_mask}, // handle the back-and-forth motion, just
  { toaster3, toaster3_mask}, // refer to frames in the desired sequence.
  { toaster3, toaster3_mask}, // This also doubles up the first and last
  { toaster2, toaster2_mask}, // frames to create a small pause at those
  { toaster1, toaster1_mask}, // points in the loop.
  { toaster0, toaster0_mask},
  { toast0  , toast_mask},    // Light,
  { toast1  , toast_mask},    // medium and
  { toast2  , toast_mask},    // dark toast - all use the same mask.
};

// 12 sprites seems to be about the limit for this code to maintain
// consistent 60 frames/sec updates. Double-buffered graphics synchronize
// to display refresh, and if something can't complete in one frame,
// everything is stalled until the next. It's especially annoying in
// edge cases with some frames take 1/60 sec but others take 1/30.
// See notes at end of file regarding potential improvements.
#define N_SPRITES 12

// This structure contains positions and other data for the sprites
// in motion (notice it's not "const", because contents change).
struct {
  int16_t pos[2]; // sprite position (X,Y) * 16
  int8_t  speed;  // sprite speed
  uint8_t frame;  // for animation
} sprite[N_SPRITES];

// Initialize one sprite (index passed to function) to a random offscreen
// position, also randomizing speed and sprite type (toaster or toast).
void randomsprite(uint8_t i) {
  // To move the sprites at slightly different speeds, coordinates are
  // stored in 1/16 pixel units. When drawing, the stored values get
  // divided by 16 to yield final pixel coordinates.
  sprite[i].pos[0] = display.width() * 16; // Off right edge
  sprite[i].pos[1] = random(-display.width() / 2, display.height()) * 16;
  sprite[i].speed = random(8, 17); // Move 1/2 to 1 pixel per frame
  // The spritedata array has 8 toaster frames and 3 toasts; just picking
  // one randomly gives us 8/11 odds of a toaster (with a random initial
  // wing position) and 3/11 of toast (with random done-ness), good mix.
  sprite[i].frame = random(sizeof spritedata / sizeof spritedata[0]);
}

void setup() { // Runs once on startup
  if (!display.begin()) { // Blink LED if insufficient RAM
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }

  // Initialize color palette from table in sprites.h. Rather than
  // calling setColor() for each one, we can just dump it directly...
  memcpy(display.getPalette(), palette, sizeof palette);
  display.swap(false, true); // Duplicate same palette to front & back buffers

  // Randomize initial sprite states
  randomSeed(analogRead(A0)); // Seed randomness from unused analog in
  for (int i=0; i<N_SPRITES; i++) randomsprite(i);
}

uint8_t flap = 0; // Counter for flapping animation

void loop() { // Runs once every frame
  display.fillScreen(0);            // Clear back framebuffer,
  for (int i=0; i<N_SPRITES; i++) { // and draw each sprite there...
    // Although DVIGFX8 is a COLOR display type, we leverage GFX's
    // drawGrayscaleBitmap() function to draw the sprites...it saves us
    // writing a ton of code this way. See notes at end of this file.
    // Also here's the "divide position by 16" mentioned earlier:
    display.drawGrayscaleBitmap(sprite[i].pos[0] / 16, sprite[i].pos[1] / 16,
      spritedata[sprite[i].frame].sprite, spritedata[sprite[i].frame].mask,
      64, 64); // All sprites are 64x64 to simplify this code
    // After drawing each sprite, update its position
    sprite[i].pos[0] -= sprite[i].speed;     // Move left
    sprite[i].pos[1] += sprite[i].speed / 2; // Move down (X:Y 2:1 ratio)
    // Every fourth video frame, IF this sprite is a toaster (frame is 0-7),
    // increment the sprite index and wrap around back to 0 if necessary...
    if ((!(flap & 3)) && (sprite[i].frame < 8)) {
      sprite[i].frame = (sprite[i].frame + 1) & 7; // Loop 0-7
    } // else is video frame 1-3 or is toast (not animated)
    // If the sprite has moved off the left or bottom edges, reassign it:
    if ((sprite[i].pos[0] < -64 * 16) || (sprite[i].pos[1] > display.height() * 16))
      randomsprite(i);
  }
  // Swap front/back buffers, do not duplicate current screen state
  // to next frame, we'll draw it new from scratch each time.
  display.swap();
  flap++; // Increment animation counter; "rolls over" 0-255 automatically.
}

/*
NOTES ON EFFICIENCY
This was written to be a silly and somewhat simple example. No care is
taken to minimize program size or speed, so it's NOT a good role model
for complex situations, but it was quick to produce. Given time and effort,
what could be improved?
- As written, every sprite is 64x64 pixels, period. Makes it very easy to
  draw. Flash space could be saved by cropping each sprite to a minimum
  bounding rectangle...tradeoff being there would need to be a table of
  sizes and X/Y offsets to maintain consistent motion when animating.
  Demo's using ~230K -- just a fraction of available flash -- and life is
  short so it's just not a priority here.
- The GFX library's drawGrayscaleBitmap() function is used because it's
  there and works for our needs, but is not especially efficient (testing
  clipping on every pixel) and expects 1 byte/pixel data. Since this demo's
  sprites are all 16-color, "packing" 2 pixels/byte is possible, using half
  as much flash. A function could be written both to handle clipping more
  efficiently and to de-pack and draw sprite data straight to framebuffer,
  but would likely increase source code 2-3X and confuse novices.
- The sprite data is all stored in flash memory, which is slower to access
  than RAM. RAM is scarce (perhaps ~64K after PicoDVI claims a framebuffer),
  but the sprites might still fit, especially if packed 2 pixels/byte. It's
  not a bottleneck now AS WRITTEN (due to drawGrayscaleBitmap() itself being
  a bit pokey), but if sprite-drawing were handled as mentioned above, could
  be possible to increase the sprite count while maintaining frame rate by
  adding the __not_in_flash attribute to these tables (see Pico SDK docs).
*/
