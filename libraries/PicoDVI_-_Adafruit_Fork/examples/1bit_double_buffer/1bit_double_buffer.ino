// Double-buffered 1-bit Adafruit_GFX-compatible framebuffer for PicoDVI.
// Animates without redraw flicker. Requires Adafruit_GFX >= 1.11.5

#include <PicoDVI.h>

// Here's how a 640x480 1-bit (black, white) framebuffer is declared.
// Second argument ('true' here) enables double-buffering for flicker-free
// animation. Third argument is a hardware configuration -- examples are
// written for Adafruit Feather RP2040 DVI, but that's easily switched out
// for boards like the Pimoroni Pico DV (use 'pimoroni_demo_hdmi_cfg') or
// Pico DVI Sock ('pico_sock_cfg').
DVIGFX1 display(DVI_RES_640x480p60, true, adafruit_feather_dvi_cfg);

// An 800x480 mode is possible but pushes overclocking even higher than
// 640x480 mode. SOME BOARDS MIGHT SIMPLY NOT BE COMPATIBLE WITH THIS.
// May require selecting QSPI div4 clock (Tools menu) to slow down flash
// accesses, may require further over-volting the CPU to 1.25 or 1.3 V.
//DVIGFX1 display(DVI_RES_800x480p60, true, adafruit_feather_dvi_cfg);

#define N_BALLS 100 // Number of bouncy balls to draw
struct {
  int16_t pos[2]; // Ball position (X,Y)
  int8_t  vel[2]; // Ball velocity (X,Y)
} ball[N_BALLS];

void setup() { // Runs once on startup
  if (!display.begin()) { // Blink LED if insufficient RAM
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
  }

  // Randomize initial ball positions and velocities
  for (int i=0; i<N_BALLS; i++) {
    ball[i].pos[0] = 10 + random(display.width() - 20);
    ball[i].pos[1] = 10 + random(display.height() - 20);
    do {
      ball[i].vel[0] = 4 - random(9);
      ball[i].vel[1] = 4 - random(9);
    } while ((ball[i].vel[0] == 0) && (ball[i].vel[1] == 0));
  }
}

void loop() {
  display.fillScreen(0); // Clear back framebuffer...
  // And draw bouncy balls (circles) there
  for (int i=0; i<N_BALLS; i++) {
    display.drawCircle(ball[i].pos[0], ball[i].pos[1], 40, 1);
    // After drawing each one, update positions, bounce off edges.
    ball[i].pos[0] += ball[i].vel[0];
    if ((ball[i].pos[0] <= 0) || (ball[i].pos[0] >= display.width())) ball[i].vel[0] *= -1;
    ball[i].pos[1] += ball[i].vel[1];
    if ((ball[i].pos[1] <= 0) || (ball[i].pos[1] >= display.height())) ball[i].vel[1] *= -1;
  }

  // Swap front/back buffers, do not duplicate current screen state to next frame,
  // we'll draw it new from scratch each time.
  display.swap();
}
