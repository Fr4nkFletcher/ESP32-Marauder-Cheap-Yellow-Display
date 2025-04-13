#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"

#include "wikimedia_christmas_tree_in_field_320x240_rgb565.h"
#include "snowflakes.h"

#define SNOWFLAKE_SIZE 64
#define N_SNOWFLAKE_IMAGES 8
#define N_SPRITES 18

#define RED_MASK 0xf800u
#define GREEN_MASK 0x07e0u
#define BLUE_MASK 0x001fu

// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

struct dvi_inst dvi0;

void core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	while (queue_is_empty(&dvi0.q_colour_valid))
		__wfe();
	dvi_start(&dvi0);
	dvi_scanbuf_main_16bpp(&dvi0);
}

// -----------------------------------------------------------------------------

#define N_SCANLINE_BUFFERS 6
uint16_t __attribute__((aligned(4))) static_scanbuf[N_SCANLINE_BUFFERS][FRAME_WIDTH];

static inline uint16_t rgb565_lerp(uint32_t c0, uint32_t c1, uint alpha) {
	return
		(((c0 & RED_MASK  ) * (256 - alpha) + (c1 & RED_MASK  ) * alpha) >> 8 & RED_MASK   ) |
		(((c0 & GREEN_MASK) * (256 - alpha) + (c1 & GREEN_MASK) * alpha) >> 8 & GREEN_MASK ) |
		(((c0 & BLUE_MASK ) * (256 - alpha) + (c1 & BLUE_MASK ) * alpha) >> 8 & BLUE_MASK  );
}

static inline uint16_t rgb565_lerp_white(uint32_t c0, uint alpha) {
	// Note alpha is preinverted (or you can think of this as "lerp from white")
	alpha += 1;
	// Since alpha is only 6 bits, we can combine the red and blue operations
	// into 1, since the zeroes in the green region break the carries.
	c0 = ~c0;
	c0 =
		(((c0 & (RED_MASK | BLUE_MASK)) * alpha >> 6) & (RED_MASK | BLUE_MASK)) |
		(((c0 & GREEN_MASK            ) * alpha >> 6) & GREEN_MASK            );
	return ~c0;
}

static inline float uniform() {
	return (float)rand() / RAND_MAX;
}

typedef struct {
	float x, y;
	float drift_x, drift_y;
	float waft;
	float waft_freq;
	float waft_amplitude;
	uint image_select;
} snowflake_t;

snowflake_t sprites[N_SPRITES];

void update() {
	for (int i = 0; i < N_SPRITES; ++i) {
		snowflake_t *s = &sprites[i];
		s->x += s->drift_x + sinf(s->waft) * s->waft_amplitude;
		s->y += s->drift_y;
		s->waft += s->waft_freq;
		if (s->y >= FRAME_HEIGHT || s->x < -SNOWFLAKE_SIZE || s->x >= FRAME_WIDTH) {
			s->y = -SNOWFLAKE_SIZE;
			s->x = uniform() * (FRAME_HEIGHT - SNOWFLAKE_SIZE);
			s->drift_x = uniform() * 2.f - 1.f;
			s->drift_y = uniform() * 1.5f + 1.f;
			s->waft = 0;
			s->waft_freq = uniform() * 0.1f + 0.01f;
			s->waft_amplitude = uniform() * 0.1f / s->waft_freq;
			sprites[i].image_select = (rand() >> 8) % N_SNOWFLAKE_IMAGES;
		}
	}
}

void __scratch_x("render") render_scanline(uint16_t *scanbuf, uint raster_y) {
	// Use DMA to copy in background line (for speed)
	uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_write_increment(&cfg, true);
    dma_channel_configure(
    	dma_chan,
    	&cfg,
    	scanbuf,
    	&((const uint16_t*)wikimedia_christmas_tree_in_field_320x240)[raster_y * FRAME_WIDTH],
    	FRAME_WIDTH / 2,
    	true
    );
    dma_channel_wait_for_finish_blocking(dma_chan);
    dma_channel_unclaim(dma_chan);

	for (int s = 0; s < N_SPRITES; ++s) {
		uint8_t *img = (uint8_t*)snowflakes + SNOWFLAKE_SIZE * SNOWFLAKE_SIZE * sprites[s].image_select;
 
 		// Check if sprite intersects scanline in y
		uint i = raster_y - (uint)(int)sprites[s].y;
		if (i >= SNOWFLAKE_SIZE)
			continue;

		#pragma GCC unroll 8
		for (int j = 0; j < SNOWFLAKE_SIZE; ++j) {
			int x = (int)sprites[s].x + j;
			if (x < 0)
				continue;
			if (x >= FRAME_WIDTH)
				break;
			scanbuf[x] = rgb565_lerp_white(
				scanbuf[x],
				img[j + SNOWFLAKE_SIZE * i]
			);
		}
	}
}

// -----------------------------------------------------------------------------

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	for (int i = 0; i < N_SCANLINE_BUFFERS; ++i) {
		void *bufptr = &static_scanbuf[i];
		queue_add_blocking((void*)&dvi0.q_colour_free, &bufptr);
	}

	// DVI will start when first valid scanbuf is presented
	multicore_launch_core1(core1_main);

    for (int i = 0; i < N_SPRITES; ++i) {
        // Initialise all sprites to out-of-bounds so update will sprinkle them from the top
        sprites[i].x = 0;
        sprites[i].y = 10 * FRAME_HEIGHT;
        sprites[i].drift_x = 0;
        sprites[i].drift_y = 0;
        sprites[i].waft = 0;
        sprites[i].waft_freq = 0;
        sprites[i].waft_amplitude = 0;
        sprites[i].image_select = 0;
    }

	uint frame_ctr = 0;
	while (true) {
		for (uint y = 0; y < FRAME_HEIGHT; ++y) {
			uint16_t *scanbuf;
			queue_remove_blocking_u32(&dvi0.q_colour_free, &scanbuf);
			render_scanline(scanbuf, y);
			queue_add_blocking_u32(&dvi0.q_colour_valid, &scanbuf);
		}
		update();
		++frame_ctr;
	}
}
