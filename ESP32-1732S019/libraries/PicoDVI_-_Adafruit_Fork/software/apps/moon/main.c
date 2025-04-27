#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/vreg.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode_1bpp.pio.h"
#include "tmds_encode.h"

// Display a full-resolution 1bpp image. By default this uses the fast 1bpp
// encode routine from libdvi (which is actually fast enough for bitplaned
// RGB111). If USE_PIO_TMDS_ENCODE is defined, the TMDS encode can be
// offloaded to a slower encode loop on a spare state machine.

// Pick one:
#define MODE_640x480_60Hz
// #define MODE_1280x720_30Hz

#if defined(MODE_640x480_60Hz)
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz
#include "moon_1bpp_640x480.h"
#define moon_img moon_1bpp_640x480

#elif defined(MODE_1280x720_30Hz)
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_1280x720p_30hz
#include "moon_1bpp_1280x720.h"
#define moon_img moon_1bpp_1280x720

#else
#error "Select a video mode!"
#endif

struct dvi_inst dvi0;

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
	setup_default_uart();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);

	// Set up extra SM, and DMA channels, to offload TMDS encode if necessary
	// (note there are two build targets for this app, called `moon` and
	// `moon_pio_encode`, so a simple `make all` will get you both binaries)
#ifdef USE_PIO_TMDS_ENCODE
	PIO encode_pio = dvi0.ser_cfg.pio;
	uint encode_sm = pio_claim_unused_sm(encode_pio, true);
	tmds_encode_1bpp_init(encode_pio, encode_sm);

	uint dma_chan_put = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(dma_chan_put);
	channel_config_set_dreq(&c, pio_get_dreq(encode_pio, encode_sm, true));
	dma_channel_configure(dma_chan_put, &c,
		&encode_pio->txf[encode_sm],
		NULL,
		FRAME_WIDTH / 32,
		false
	);

	uint dma_chan_get = dma_claim_unused_channel(true);
	c = dma_channel_get_default_config(dma_chan_get);
	channel_config_set_dreq(&c, pio_get_dreq(encode_pio, encode_sm, false));
	channel_config_set_write_increment(&c, true);
	channel_config_set_read_increment(&c, false);
	dma_channel_configure(dma_chan_get, &c,
		NULL,
		&encode_pio->rxf[encode_sm],
		FRAME_WIDTH / DVI_SYMBOLS_PER_WORD,
		false
	);
#endif

	dvi_start(&dvi0);
	while (true) {
		for (uint y = 0; y < FRAME_HEIGHT; ++y) {
			const uint32_t *colourbuf = &((const uint32_t*)moon_img)[y * FRAME_WIDTH / 32];
			uint32_t *tmdsbuf;
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);
#ifndef USE_PIO_TMDS_ENCODE
			tmds_encode_1bpp(colourbuf, tmdsbuf, FRAME_WIDTH);
#else
			dma_channel_set_read_addr(dma_chan_put, colourbuf, true);
			dma_channel_set_write_addr(dma_chan_get, tmdsbuf, true);
			dma_channel_wait_for_finish_blocking(dma_chan_get);
#endif
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
		}
	}
}
