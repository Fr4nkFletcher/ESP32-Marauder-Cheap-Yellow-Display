// "TV host" example for PicoDVI library. If just starting out,
// see the 8bit_double_buffer which explains the PicoDVI groundwork.
// Comments in THIS file are mostly distinct & new concepts.

// IF NO OUTPUT OR RED FLICKER SCANLINES: try Tools->Optimize->(-O3)

#include <PicoDVI.h>
#include "sprites.h" // Graphics data

DVIGFX8 display(DVI_RES_320x240p60, true, adafruit_feather_dvi_cfg);

// See notes in 8bit_double_buffer regarding 400x240 mode.
//DVIGFX8 display(DVI_RES_400x240p60, true, adafruit_feather_dvi_cfg);
// Also requires -O3 setting.

#define SPRITE_WIDTH  256
#define SPRITE_HEIGHT 207

// This structure holds pointers to sprite graphics and masks in sprites.h.
const struct {
  const uint8_t *sprite;
  const uint8_t *mask;
} spritedata[] = {
  { sprite0 , mask0  },
  { sprite1 , mask1  },
  { sprite2 , mask2  },
  { sprite3 , mask3  },
  { sprite4 , mask4  },
  { sprite5 , mask5  },
  { sprite6 , mask6  },
  { sprite7 , mask7  },
  { sprite8 , mask8  },
  { sprite9 , mask9  },
  { sprite10, mask10 },
  { sprite11, mask11 },
  { sprite12, mask12 },
  { sprite13, mask13 },
  { sprite14, mask14 },
  { sprite15, mask15 },
  { sprite16, mask16 }
};
#define NUM_SPRITES (sizeof spritedata / sizeof spritedata[0])

// Spinning cube implements 3D rotation & projection, but takes every
// mathematical shortcut; not recommended as a starting point for proper
// 3D drawing. Check out Wikipedia, Foley & Van Dam, etc.
#define STEPS 20   // Number of lines between cube edges
#define CAM_Z 1.8  // Camera position along Z axis; approx sqrt(3) * 1.05
float scale;       // Calc'd once per frame for in/out bounce effect
float min_scale = sqrt((display.width() * display.width() + display.height() * display.height()) / 4) / M_SQRT2 * 2.0;

void setup() { // Runs once on startup
  if (!display.begin()) { // Blink LED if insufficient RAM
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }

  // Initialize color palette from table in sprites.h. Rather than
  // calling setColor() for each one, we can just dump it directly...
  memcpy(display.getPalette(), palette, sizeof palette);
  // And then override the last few entries for our background cube...
  display.setColor(0, 0, 0, 0);       // Black background
  display.setColor(253, 255, 0, 255); // Index 253 = magenta
  display.setColor(254, 0, 255, 255); // Index 254 = cyan
  display.setColor(255, 255, 255, 0); // Index 255 = yellow
  display.swap(false, true); // Duplicate same palette into front & back buffers
}

const float corners[8][3] = { // Cube vertices in original coord space
  { -1, -1, -1 },
  {  1, -1, -1 },
  { -1,  1, -1 },
  {  1,  1, -1 },
  { -1, -1,  1 },
  {  1, -1,  1 },
  { -1,  1,  1 },
  {  1,  1,  1 },
};

float rotated[8][3]; // Cube vertices after rotation applied
// (Cam is stationary, cube is transformed, rather than other way around)

// Draw one face of the cube, if it's visible (not back-facing).
// Some points & doubled-up lines could be avoided with more care,
// but code would be somewhat longer. Not a bottleneck here, so...
void face(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t color) {
  // p1-p4 are indices of 4 corners, p5 is a perpendicular point (surface normal)
  float vx = rotated[p1][0];                  // Vector from cam to first point
  float vy = rotated[p1][1];                  // (not normalized)
  float vz = rotated[p1][2] - CAM_Z;
  float nx = rotated[p5][0] - rotated[p1][0]; // Surface normal
  float ny = rotated[p5][1] - rotated[p1][1]; // (not normalized)
  float nz = rotated[p5][2] - rotated[p1][2];
  // Dot product of vectors. Since only the sign of the result is needed
  // below, the two vectors do not require normalization first.
  float dot = vx * nx + vy * ny + vz * nz;
  if (dot < 0) {                 // Facing camera?
    for(int i=0; i<STEPS; i++) { // Interpolate parallel lines...
      float s1 = (float)i / (float)(STEPS - 1); // Weighting of p1, p3
      float s2 = 1.0 - s1;                      // Weighting of p2, p4
      // Interpolate between p1, p2
      float t1x = rotated[p1][0] * s1 + rotated[p2][0] * s2;
      float t1y = rotated[p1][1] * s1 + rotated[p2][1] * s2;
      float t1z = rotated[p1][2] * s1 + rotated[p2][2] * s2;
      // Interpolate between p3, p4
      float t2x = rotated[p3][0] * s1 + rotated[p4][0] * s2;
      float t2y = rotated[p3][1] * s1 + rotated[p4][1] * s2;
      float t2z = rotated[p3][2] * s1 + rotated[p4][2] * s2;
      // Project interpolated endpoints to image plane @ Z=0
      float dz = t1z - CAM_Z; // Camera to point
      float x1 = (float)(display.width()  / 2) + t1x / dz * scale + 0.5;
      float y1 = (float)(display.height() / 2) + t1y / dz * scale + 0.5;
      dz = t2z - CAM_Z;       // Camera to point
      float x2 = (float)(display.width()  / 2) + t2x / dz * scale + 0.5;
      float y2 = (float)(display.height() / 2) + t2y / dz * scale + 0.5;
      line(x1, y1, x2, y2, color);
    }
  }
}

uint8_t  frame = 0;         // For animation timing
bool     stutter = false;   // If true, doing A/B stutter animation
uint32_t duration = 3000;   // Time to hold current mode, in milliseconds
uint32_t lastModeSwitchTime = 0;
uint8_t  var[2] = { 4, 4 }; // Mode-specific variables
uint8_t  spritenum = 0;     // Active sprite index

void loop() { // Runs once every frame
  display.fillScreen(0); // Clear back framebuffer, will do full redraw...

  // This does the in & out bounce effect (bit of a cheat but less math
  // than moving camera in & out. Cam stays in one position just far
  // enough that it's never inside cube, need not handle Z clipping.)
  scale = min_scale * (1.0 + (float)abs((long)(millis() & 4095) - 2048) / 8192.0); // 1.0-1.25X

  // Rotate cube vertices from corners[] to rotated[]
  float now = millis() / 1000.0; // Elapsed time in seconds
  float rx = now * -0.19;        // X, Y, Z rotation
  float ry = now *  0.23;
  float rz = now *  0.13;
  float cx = cos(rx);
  float cy = cos(ry);
  float cz = cos(rz);
  float sx = sin(rx);
  float sy = sin(ry);
  float sz = sin(rz);
  for (int i=0; i<8; i++) {
    float x1 = corners[i][0];
    float y1 = corners[i][1];
    float z1 = corners[i][2];
    float y2 = y1 * cx - z1 * sx;
    float z2 = y1 * sx + z1 * cx;
    float x2 = x1 * cy - z2 * sy;
    float z3 = x1 * sy + z2 * cy;
    float x3 = x2 * cz - y2 * sz;
    float y3 = x2 * sz + y2 * cz;
    rotated[i][0] = x3;
    rotated[i][1] = y3;
    rotated[i][2] = z3;
  }

  face(0, 1, 4, 5, 2, 253); // Test & draw 6 faces of cube
  face(0, 2, 4, 6, 1, 254);
  face(2, 3, 6, 7, 0, 253);
  face(1, 3, 5, 7, 0, 254);
  face(0, 1, 2, 3, 4, 255);
  face(4, 5, 6, 7, 0, 255);

  // Check if switching in/out of stutter mode
  if ((millis() - lastModeSwitchTime) >= duration) { // Time to switch?
    lastModeSwitchTime = millis();   // Note time,
    stutter = !stutter;              // and make the switch
    if (stutter) {                   // If entering stutter mode...
      duration = random(250, 500);   // Go for 1/4 to 1/2 second
      var[0] = random(NUM_SPRITES);  // Pick 2 different sprites
      var[1] = (var[0] + 1 + random(NUM_SPRITES - 1)) % NUM_SPRITES;
    } else {                         // Returning to regular mode...
      duration = random(2000, 5000); // Go for 2-5 seconds
      var[0] = random(3, 6);         // Hold each pose for 3-5 frames
    }
  }

  // Choose sprite to display based on current mode...
  if (stutter) {
    // Every second frame, alternate between two sprites
    spritenum = var[((frame >> 1) & 1)];
  } else {
    // Pick among random frames (may repeat, is OK)
    if (!(frame % var[0])) {
      spritenum = random(NUM_SPRITES);
    }
  }

  // Overlay big sprite. Although DVIGFX8 is a COLOR display type,
  // we leverage GFX's drawGrayscaleBitmap() function to draw it...
  // saves us writing a ton of code this way.
  // Comment this out if you want ONLY the background cube.
  display.drawGrayscaleBitmap((display.width() - SPRITE_WIDTH) / 2,
    display.height() - SPRITE_HEIGHT, spritedata[spritenum].sprite,
    spritedata[spritenum].mask, SPRITE_WIDTH, SPRITE_HEIGHT);

  // Swap front/back buffers, do not duplicate current screen state
  // to next frame, we'll draw it new from scratch each time.
  display.swap();
  frame++; // For animation timing; wraps around 0-255
}

// A somewhat smooth(er) line drawing function. Not antialiased, but with
// subpixel positioning that makes the background animation less "jumpy"
// than with GFX, which is all integer end points.

// Returns bitmask of which edge(s) a point exceeds
uint8_t xymask(float x, float y) {
  return ( (x < 0.0      ) | ((x >= (float)display.width() ) << 1) |
          ((y < 0.0) << 2) | ((y >= (float)display.height()) << 3));
}

// Returns bitmask of which X edge(s) a point exceeds
uint8_t xmask(float x) {
  return (x < 0.0) | ((x >= (float)display.width()) << 1);
}

void line(float x1, float y1, float x2, float y2, uint8_t color) {
float ox1 = x1, oy1 = y1, ox2 = x2, oy2 = y2;
  uint8_t mask1 = xymask(x1, y1); // If both endpoints are
  uint8_t mask2 = xymask(x2, y2); // off same edge(s),
  if (mask1 & mask2) return;      // line won't overlap screen

  float dx = x2 - x1;
  float dy = y2 - y1;

  // Clip top
  if (mask1 & 4) {
    x1 -= y1 * dx / dy;
    y1  = 0.0;
  } else if (mask2 & 4) {
    x2 -= y2 * dx / dy;
    y2  = 0.0;
  }

  float maxy = (float)(display.height() - 1);

  // Clip bottom
  if (mask1 & 8) {
    x1 -= (y1 - maxy) * dx / dy;
    y1  = maxy;
  } else if (mask2 & 8) {
    x2 -= (y2 - maxy) * dx / dy;
    y2  = maxy;
  }

  mask1 = xmask(x1); // Repeat the offscreen check after Y clip
  mask2 = xmask(x2);
  if (mask1 & mask2) return;

  dx = x2 - x1;
  dy = y2 - y1;

  // Clip left
  if (mask1 & 1) {
    y1 -= x1 * dy / dx;
    x1  = 0.0;
  } else if (mask2 & 1) {
    y2 -= x2 * dy / dx;
    x2  = 0.0;
  }

  float maxx = (float)(display.width() - 1);

  // Clip right
  if (mask1 & 2) {
    y1 -= (x1 - maxx) * dy / dx;
    x1  = maxx;
  } else if (mask2 & 2) {
    y2 -= (x2 - maxx) * dy / dx;
    x2  = maxx;
  }

  // (x1, y1) to (x2, y2) is known to be on-screen and in-bounds now.

  // Handle a couple special cases (horizontal, vertical lines) first,
  // GFX takes care of these fine and it avoids some divide-by-zero
  // checks in the diagonal code later.
  if ((int)y1 == (int)y2) { // Horizontal or single point
    int16_t x, w;
    if (x2 >= x1) {
      x = (int)x1;
      w = (int)x2 - (int)x1 + 1;
    } else {
      x = (int)x2;
      w = (int)x1 - (int)x2 + 1;
    }
    display.drawFastHLine(x, (int)y1, w, color);
  } else if ((int)x1 == (int)x2) { // Vertical
    int16_t y, h;
    if (y2 >= y1) {
      y = (int)y1;
      h = (int)y2 - (int)y1 + 1;
    } else {
      y = (int)y2;
      h = (int)y1 - (int)y2 + 1;
    }
    display.drawFastVLine((int)x1, y, h, color);
  } else { // Diagonal
    dx = x2 - x1;
    dy = y2 - y1;

    uint8_t *ptr = display.getBuffer();

    // This is a bit ugly in that it uses floating-point math in the line
    // drawing loop. There are more optimal Bresenham-esque fixed-point
    // approaches to do this (initializing the error term based on subpixel
    // endpoints and slopes), but A) I'm out of spoons today, and B) we're
    // drawing just a few dozen lines and it's simply not a bottleneck in
    // this demo. Just saying this won't scale up to thousands of lines.

    if (fabs(dx) >= fabs(dy)) { // "Wide" line
      if (x1 > x2) {            // Put (x1, y1) at left
        float t = x1; x1 = x2; x2 = t;
        t = y1; y1 = y2; y2 = t;
      }
      uint16_t ix1   = (uint16_t)x1;
      uint16_t ix2   = (uint16_t)x2;
      float    slope = dy / dx;
      for (uint16_t x=ix1; x <= ix2; x++) {
        uint16_t iy = (uint16_t)(y1 + slope * (float)(x - ix1));
        if (iy < display.height()) ptr[iy * display.width() + x] = color;
      }
    } else {         // "Tall" line
      if (y1 > y2) { // Put (x1, y1) at top
        float t = x1; x1 = x2; x2 = t;
        t = y1; y1 = y2; y2 = t;
      }
      uint16_t iy1   = (uint16_t)y1;
      uint16_t iy2   = (uint16_t)y2;
      float    slope = dx / dy;
      for (uint16_t y=iy1; y <= iy2; y++) {
        uint16_t ix = (uint16_t)(x1 + slope * (float)(y - iy1));
        if (ix < display.width()) ptr[y * display.width() + ix] = color;
      }
    }
  }
}
