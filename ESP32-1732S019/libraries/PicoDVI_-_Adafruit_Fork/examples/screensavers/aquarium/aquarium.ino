// "Aquarium" example for PicoDVI library. If just starting out,
// see the 8bit_double_buffer which explains the PicoDVI groundwork.
// Comments in THIS file are mostly distinct & new concepts.
// The flying toasters example also goes into more detail.

// IF NO OUTPUT OR RED FLICKER SCANLINES: try Tools->Optimize->(-O3)

#include <PicoDVI.h>
#include "sprites.h" // Graphics data

DVIGFX8 display(DVI_RES_320x240p60, true, adafruit_feather_dvi_cfg);

// See notes in 8bit_double_buffer regarding 400x240 mode.
//DVIGFX8 display(DVI_RES_400x240p60, true, adafruit_feather_dvi_cfg);
// Also requires -O3 setting.

// This structure holds pointers to sprite graphics and masks in sprites.h.
const struct {
  const uint8_t *sprite[2][2]; // L/R directions and A/B frames
  const uint8_t *mask[2][2];   // Same
} spritedata[] = {
  // There are FOUR sprites/masks for each fish (and kelp):
  // two left-facing, two right-facing, and A/B frames for each.
  { sprite0LA , sprite0LB , sprite0RA , sprite0RB , mask0LA , mask0LB , mask0RA , mask0RB  },
  { sprite1LA , sprite1LB , sprite1RA , sprite1RB , mask1LA , mask1LB , mask1RA , mask1RB  },
  { sprite2LA , sprite2LB , sprite2RA , sprite2RB , mask2LA , mask2LB , mask2RA , mask2RB  },
  { sprite3LA , sprite3LB , sprite3RA , sprite3RB,  mask3LA , mask3LB , mask3RA , mask3RB  },
  { sprite4LA , sprite4LB , sprite4RA , sprite4RB , mask4LA , mask4LB , mask4RA , mask4RB  },
  { sprite5LA , sprite5LB , sprite5RA , sprite5RB , mask5LA , mask5LB , mask5RA , mask5RB  },
  { sprite6LA , sprite6LB , sprite6RA , sprite6RB , mask6LA , mask6LB , mask6RA , mask6RB  },
  { sprite7LA , sprite7LB , sprite7RA , sprite7RB , mask7LA , mask7LB , mask7RA , mask7RB  },
  { sprite8LA , sprite8LB , sprite8RA , sprite8RB , mask8LA , mask8LB , mask8RA , mask8RB  },
  { sprite9LA , sprite9LB , sprite9RA , sprite9RB , mask9LA , mask9LB , mask9RA , mask9RB  },
  { sprite10LA, sprite10LB, sprite10RA, sprite10RB, mask10LA, mask10LB, mask10RA, mask10RB },
  { sprite11LA, sprite11LB, sprite11RA, sprite11RB, mask11LA, mask11LB, mask11RA, mask11RB },
  { sprite12LA, sprite12LB, sprite12RA, sprite12RB, mask12LA, mask12LB, mask12RA, mask12RB },
  { sprite13LA, sprite13LB, sprite13RA, sprite13RB, mask13LA, mask13LB, mask13RA, mask13RB },
  { sprite14LA, sprite14LB, sprite14RA, sprite14RB, mask14LA, mask14LB, mask14RA, mask14RB },
  { sprite15LA, sprite15LB, sprite15RA, sprite15RB, mask15LA, mask15LB, mask15RA, mask15RB },
  { sprite16LA, sprite16LB, sprite16RA, sprite16RB, mask16LA, mask16LB, mask16RA, mask16RB },
  { sprite17LA, sprite17LB, sprite17RA, sprite17RB, mask17LA, mask17LB, mask17RA, mask17RB },
  // Bubbles are a special case. No right/left versions, but A/B frames.
  // To use same struct (not a special case), just duplicate each 2X.
  { sprite18A , sprite18B , sprite18A , sprite18B , mask18A , mask18B , mask18A , mask18B  },
};

#define N_SPRITES 12 // MUST be >= 6

// This structure contains positions and other data for the sprites
// in motion (notice it's not "const", because contents change).
struct {
  int16_t pos[2]; // sprite position (X,Y) * 16
  int8_t  speed;  // sprite speed (-16 to -8 or +8 to +16)
  uint8_t index;  // which index (in spritedata) to use
  uint8_t offset; // Timer offset to de-sync each sprite's animation
} sprite[N_SPRITES];

// Initialize one sprite (index passed to function) to a random offscreen
// position, also randomizing speed and sprite (fish) type.
void randomsprite(uint8_t i) {
  // To move the sprites at slightly different speeds, coordinates are
  // stored in 1/16 pixel units. When drawing, the stored values get
  // divided by 16 to yield final pixel coordinates.
  sprite[i].speed = random(8, 17); // 1/2 to 1 pixel per frame
  if (random(2)) {                 // 50/50 random chance...
    sprite[i].speed *= -1;         // Fish moves right-to-left
    sprite[i].pos[0] = (display.width() + random(64)) * 16; // Start off right edge
  } else {                         // Fish moves left-to-right
    sprite[i].pos[0] = random(64, 128) * -16; // Start off left edge
  }
  // WHEEL. OF. FISH. -2 here is to ignore last 2 sprites (kelp, bubbles)
  sprite[i].index = random(sizeof spritedata / sizeof spritedata[0] - 2);
  if (sprite[i].index == 8) { // Sprite #8 is crab, keep close to ground
    sprite[i].pos[1] = random(display.height() - 96, display.height() - 64) * 16;
  } else { // Is a fish, upper part of screen
    sprite[i].pos[1] = random(display.height() - 96) * 16;
  }
  sprite[i].offset = random(256); // De-synchronize sprite animation
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
  int range = display.width() + 64;
  for (int i=0; i<3; i++) {   // FIRST THREE sprites...
    sprite[i].index = 17;     // Are always kelp
    sprite[i].speed = random(2) ? 1 : -1; // 50/50 left/right flip
    sprite[i].pos[0] = (random(range * i / 3, range * (i + 1) / 3 - 64) - 32) * 16;
    sprite[i].pos[1] = random(display.height() - 120, display.height() - 100) * 16;
    sprite[i].offset = random(256);
  }
  for (int i=3; i<6; i++) { // NEXT THREE sprites...
    sprite[i].index = 18;   // Are always bubbles
    sprite[i].speed = 0;
    sprite[i].pos[0] = display.width() * 16; // Start them all offscreen
    sprite[i].pos[1] = random(display.height()) * 8;
    sprite[i].offset = random(256);
  }
  for (int i=6; i<N_SPRITES; i++) randomsprite(i); // Rest are fish
}

uint8_t frame = 0; // Counter for animation

void loop() { // Runs once every frame
  display.fillScreen(0);                       // Clear back framebuffer,
  for (int x=0; x<display.width(); x += 192) { // Tile background sprite
    // Although DVIGFX8 is a COLOR display type, we leverage GFX's
    // drawGrayscaleBitmap() function to draw the sprites...it saves us
    // writing a ton of code this way.
    display.drawGrayscaleBitmap(x, display.height() - 64, gravel, 192, 64);
  }

  for (int i=0; i<N_SPRITES; i++) { // and then the rest of the sprites...
    uint8_t dir = sprite[i].speed > 0;                  // Left/right
    uint8_t fr = ((frame + sprite[i].offset) >> 4) & 1; // A/B frame
    if (sprite[i].speed) { // FISH or KELP; 64x64 sprite
      display.drawGrayscaleBitmap(sprite[i].pos[0] / 16, sprite[i].pos[1] / 16,
        spritedata[sprite[i].index].sprite[dir][fr],
        spritedata[sprite[i].index].mask[dir][fr], 64, 64);
      if (abs(sprite[i].speed) > 1) {        // Not kelp...
        sprite[i].pos[0] += sprite[i].speed; // Update position, check if offscreen...
        if (((sprite[i].speed > 0) && (sprite[i].pos[0] > (display.width() * 16))) ||
            ((sprite[i].speed < 0) && (sprite[i].pos[0] < -64 * 16)))
          randomsprite(i); // Replace with a new fish
      }
    } else { // Is BUBBLES
      display.drawGrayscaleBitmap(sprite[i].pos[0] / 16, sprite[i].pos[1] / 16,
        spritedata[sprite[i].index].sprite[0][fr],
        spritedata[sprite[i].index].mask[0][fr], 64, 16);
      sprite[i].pos[1] -= 16;                // Move up by 1 pixel
      if (sprite[i].pos[1] < -256) {         // Off top of screen?
        int j = random(6, N_SPRITES);        // Pick a random fish,
        sprite[i].pos[0] = sprite[j].pos[0]; // and move bubbles there
        sprite[i].pos[1] = sprite[j].pos[1] + 384;
      }
    }
  }

  // Swap front/back buffers, do not duplicate current screen state
  // to next frame, we'll draw it new from scratch each time.
  display.swap();
  frame++; // Increment animation counter; "rolls over" 0-255 automatically.
}
