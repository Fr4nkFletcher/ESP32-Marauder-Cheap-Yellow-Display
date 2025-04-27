#include <stdio.h>
#include <stdlib.h>
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
#include "tmds_encode.h"

#include "testcard_display1_rgb332.h"
#include "testcard_display2_rgb332.h"

// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz


void display_scrolling_testcard(struct dvi_inst *inst, const uint8_t *img) {
	const uint red_msb   = 7;
	const uint red_lsb   = 5;
	const uint green_msb = 4;
	const uint green_lsb = 2;
	const uint blue_msb  = 1;
	const uint blue_lsb  = 0;
	uint pixwidth = inst->timing->h_active_pixels;
	uint frame_ctr = 0;
	while (true) {
		for (uint y = 0; y < FRAME_HEIGHT; ++y) {
			uint y_scroll = (y + frame_ctr) % FRAME_HEIGHT;
			const uint8_t *colourbuf = &((const uint8_t*)img)[y_scroll * FRAME_WIDTH];
			uint32_t *tmdsbuf;
			queue_remove_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
			// NB the scanline buffers are half-resolution!
			tmds_encode_data_channel_8bpp((const uint32_t*)colourbuf, tmdsbuf, pixwidth / 2, blue_msb, blue_lsb);
			tmds_encode_data_channel_8bpp((const uint32_t*)colourbuf, tmdsbuf + pixwidth, pixwidth / 2, green_msb, green_lsb);
			tmds_encode_data_channel_8bpp((const uint32_t*)colourbuf, tmdsbuf + 2 * pixwidth, pixwidth / 2, red_msb, red_lsb);
			queue_add_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
		}
		++frame_ctr;
	}
}

struct dvi_inst dvi0;
struct dvi_inst dvi1;

void core1_main() {
	dvi_register_irqs_this_core(&dvi1, DMA_IRQ_1);
	dvi_start(&dvi1);
	display_scrolling_testcard(&dvi1, (const uint8_t*)testcard_display2);
}

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	// Run system at TMDS bit clock
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = picodvi_dvi_cfg;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	dvi1.timing = &DVI_TIMING;
	dvi1.ser_cfg = picodvi_pmod0_cfg;
	dvi1.ser_cfg.pio = pio1;
	dvi_init(&dvi1, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	multicore_launch_core1(core1_main);
	sleep_ms(10);

	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
	display_scrolling_testcard(&dvi0, (const uint8_t*)testcard_display1);
}
