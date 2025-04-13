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
#include "tile.h"

#include "map_test_zelda_mini.h"
#include "oga_zelda_overworld_flat_rgab5515.h"

// Pick one:
#define MODE_640x480_60Hz
// #define MODE_800x480_60Hz
// #define MODE_800x600_60Hz
// #define MODE_960x540p_60Hz
// #define MODE_1280x720_30Hz

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

void __not_in_flash("render") render_loop() {
	uint heartbeat = 0;
	uint frame_ctr = 0;

	tilebg_t bg = {
		.xscroll = 0,
		.yscroll = 0,
		.tileset = oga_zelda_overworld_flat,
		.tilemap = map_test_zelda_mini,
		.log_size_x = 10,                 // 1024px, 64 tiles
		.log_size_y = 9,                  // 512px,  32 tiles
		.tilesize = TILESIZE_16,
		.fill_loop = (tile_loop_t)tile16_16px_alpha_loop
	};

	while (1) {
		if (++heartbeat >= 30) {
			heartbeat = 0;
			gpio_xor_mask(1u << LED_PIN);
		}
		for (uint y = 0; y < FRAME_HEIGHT; ++y) {
			uint16_t *pixbuf;
			queue_remove_blocking(&dvi0.q_colour_free, &pixbuf);
			tile16(pixbuf, &bg, y, FRAME_WIDTH);
			queue_add_blocking(&dvi0.q_colour_valid, &pixbuf);
		}
		bg.xscroll += 3;
		bg.yscroll += 2;
		++frame_ctr;
	}
}

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

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

	// Core 1 will fire up the DVI once it sees the first colour buffer has been rendered
	printf("Start rendering\n");
	render_loop();
	__builtin_unreachable();
}
	
