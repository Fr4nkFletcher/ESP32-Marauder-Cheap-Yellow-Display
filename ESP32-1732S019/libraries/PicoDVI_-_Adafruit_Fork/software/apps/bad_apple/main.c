#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "rle_decompress.h"

#include "test_frame.h"

#define MOVIE_BASE (XIP_BASE + 0x10000)
#define MOVIE_FRAMES 209

// DVDD 1.25V (slower silicon may need the full 1.3, or just not work)
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_1280x720p_30hz

struct dvi_inst dvi0;

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	// Run system at TMDS bit clock
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);

	// Fill all of DVI's TMDS buffers with valid DC-balanced symbol pairs (mid-grey),
	// as we'll only be rendering into the middle part of each span
	uint32_t *buf;
	const uint32_t two_0x80_symbols = 0x5fd80u;
	while (queue_try_remove_u32(&dvi0.q_tmds_free, &buf)) {
		for (int i = 0; i < FRAME_WIDTH / DVI_SYMBOLS_PER_WORD; ++i)
			buf[i] = two_0x80_symbols;
		queue_add_blocking_u32(&dvi0.q_tmds_valid, &buf);
	}
	// Shuffle them back to the free queue so we don't have to deal with the timing later
	while (queue_try_remove_u32(&dvi0.q_tmds_valid, &buf))
		queue_add_blocking_u32(&dvi0.q_tmds_free, &buf);

	dvi_start(&dvi0);

	int frame = 0;
	const uint8_t *line = (const uint8_t*)MOVIE_BASE;
	while (true) {
		uint32_t *render_target;
		for (int y = 0; y < FRAME_HEIGHT; ++y) {
			uint8_t line_len = *line++;
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &render_target);
			rle_to_tmds(line, render_target + (1280 - 960) / 2 / DVI_SYMBOLS_PER_WORD, line_len);
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &render_target);
			line += line_len;
		}
		if (++frame == MOVIE_FRAMES) {
			frame = 0;
			line = (const uint8_t *)MOVIE_BASE;
		}
	}
}
	
