#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "pico/sem.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "sprite.h"

// Pick one:
#define MODE_640x480_60Hz
// #define MODE_800x480_60Hz
// #define MODE_800x600_60Hz
// #define MODE_960x540p_60Hz
// #define MODE_1280x720_30Hz

#include "raspberry_128x128_rgab5515.h"
#include "eben_128x128_rgab5515.h"

#if defined(MODE_640x480_60Hz)
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#elif defined(MODE_800x480_60Hz)
#define FRAME_WIDTH 400
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_800x480p_60hz

#elif defined(MODE_800x600_60Hz)
// DVDD 1.3V, going downhill with a tailwind
#define FRAME_WIDTH 400
#define FRAME_HEIGHT 300
#define VREG_VSEL VREG_VOLTAGE_1_30
#define DVI_TIMING dvi_timing_800x600p_60hz

#elif defined(MODE_960x540p_60Hz)
// DVDD 1.25V (slower silicon may need the full 1.3, or just not work)
// Frame resolution is almost the same as a PSP :)
#define FRAME_WIDTH 480
#define FRAME_HEIGHT 270
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_960x540p_60hz

#elif defined(MODE_1280x720_30Hz)
// 1280x720p 30 Hz (nonstandard)
// DVDD 1.25V (slower silicon may need the full 1.3, or just not work)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 360
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_1280x720p_30hz

#else
#error "Select a video mode!"
#endif

#define N_BERRIES 65
#define LED_PIN 21

struct dvi_inst dvi0;

void core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	while (queue_is_empty(&dvi0.q_colour_valid))
		__wfe();
	dvi_start(&dvi0);
	dvi_scanbuf_main_16bpp(&dvi0);
	__builtin_unreachable();
}

static inline int clip(int x, int min, int max) {
	return x < min ? min : x > max ? max : x;
}

#define N_SCANLINE_BUFFERS 4
uint16_t static_scanbuf[N_SCANLINE_BUFFERS][FRAME_WIDTH];

sprite_t berry[N_BERRIES];
int vx[N_BERRIES];
int vy[N_BERRIES];
int vt[N_BERRIES];
uint8_t theta[N_BERRIES];
affine_transform_t atrans[N_BERRIES];

const int xmin = -100;
const int xmax = FRAME_WIDTH - 30;
const int ymin = -100;
const int ymax = FRAME_HEIGHT - 30;
const int vmax = 4;

void __not_in_flash("render") render_loop() {
	uint heartbeat = 0;
	uint frame_ctr = 0;
	while (1) {
		if (++heartbeat >= 30) {
			heartbeat = 0;
			gpio_xor_mask(1u << LED_PIN);
		}
		for (uint y = 0; y < FRAME_HEIGHT; ++y) {
			uint16_t *pixbuf;
			queue_remove_blocking(&dvi0.q_colour_free, &pixbuf);
			// sprite_blit16(pixbuf, (const uint16_t *)testcard_320x240 + (y + frame_ctr / 2) % 240 * FRAME_WIDTH, 320);
			sprite_fill16(pixbuf, 0x07ff, FRAME_WIDTH);
			for (int i = 0; i < N_BERRIES; ++i)
				// sprite_asprite16(pixbuf, &berry[i], atrans[i], y, FRAME_WIDTH);
				sprite_sprite16(pixbuf, &berry[i], y, FRAME_WIDTH);
			queue_add_blocking(&dvi0.q_colour_valid, &pixbuf);
		}
		// Update during vblank
		for (int i = 0; i < N_BERRIES; ++i) {
			berry[i].x += vx[i];
			berry[i].y += vy[i];
			theta[i] += vt[i];
			affine_identity(atrans[i]);
			affine_scale(atrans[i], 7 * AF_ONE / 8, 7 * AF_ONE / 8);
			affine_translate(atrans[i], -56, -56);
			affine_rotate(atrans[i], theta[i]);
			affine_translate(atrans[i], 60, 60);
			int xclip = clip(berry[i].x, xmin, xmax);
			int yclip = clip(berry[i].y, ymin, ymax);
			if (xclip != berry[i].x || yclip != berry[i].y) {
				berry[i].x = xclip;
				berry[i].y = yclip;
				vx[i] = (rand() % vmax + 1) * (rand() & 0x8000 ? 1 : -1);
				vy[i] = (rand() % vmax + 1) * (rand() & 0x8000 ? 1 : -1);
				vt[i] = (rand() % vmax + 1) * (rand() & 0x8000 ? 1 : -1);
				berry[i].hflip = vx[i] < 0;
				berry[i].vflip = vy[i] < 0;
			}
		}
		++frame_ctr;
	}
}

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
#ifdef RUN_FROM_CRYSTAL
	// Slow everything down uniformly, so signals are probeable but the code runs
	// identically (note this actually uses the PLL with low feedback and max PD1/PD2)
	set_sys_clock_khz(12000, true);
#else
	// Run system at TMDS bit clock
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
#endif

	setup_default_uart();

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	printf("Core 1 start\n");
	multicore_launch_core1(core1_main);

	printf("Allocating scanline buffers\n");
	for (int i = 0; i < N_SCANLINE_BUFFERS; ++i) {
		void *bufptr = &static_scanbuf[i];
		queue_add_blocking((void*)&dvi0.q_colour_free, &bufptr);
	}

	for (int i = 0; i < N_BERRIES; ++i) {
		berry[i].x = rand() % (xmax - xmin + 1) + xmin;
		berry[i].y = rand() % (ymax - ymin + 1) + ymin;
		berry[i].img = i % 2 ? eben_128x128 : raspberry_128x128;
		berry[i].log_size = 7;
		berry[i].has_opacity_metadata = true; // Much faster non-AT blitting
		berry[i].hflip = false;
		berry[i].vflip = false;
		vx[i] = (rand() % vmax + 1) * (rand() & 0x8000 ? 1 : -1);
		vy[i] = (rand() % vmax + 1) * (rand() & 0x8000 ? 1 : -1);
		vt[i] = (rand() % vmax + 1) * (rand() & 0x8000 ? 1 : -1);
		theta[i] = 0;
		affine_identity(atrans[i]);
	}

	// Core 1 will fire up the DVI once it sees the first colour buffer has been rendered
	printf("Start rendering\n");
	render_loop();
	__builtin_unreachable();
}
	
