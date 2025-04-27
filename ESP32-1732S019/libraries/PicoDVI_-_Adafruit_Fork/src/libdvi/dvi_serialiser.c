#include "pico.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/structs/padsbank0.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "dvi_serialiser.pio.h"

static void dvi_configure_pad(uint gpio, bool invert) {
	// 2 mA drive, enable slew rate limiting (this seems fine even at 720p30, and
	// the 3V3 LDO doesn't get warm like when turning all the GPIOs up to 11).
	// Also disable digital receiver.
	hw_write_masked(
		&padsbank0_hw->io[gpio],
		(0 << PADS_BANK0_GPIO0_DRIVE_LSB),
		PADS_BANK0_GPIO0_DRIVE_BITS | PADS_BANK0_GPIO0_SLEWFAST_BITS | PADS_BANK0_GPIO0_IE_BITS
	);
	gpio_set_outover(gpio, invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
}

void dvi_serialiser_init(struct dvi_serialiser_cfg *cfg) {
#if DVI_SERIAL_DEBUG
	uint offset = pio_add_program(cfg->pio, &dvi_serialiser_debug_program);
#else
	uint offset = pio_add_program(cfg->pio, &dvi_serialiser_program);
#endif
	cfg->prog_offs = offset;

	for (int i = 0; i < N_TMDS_LANES; ++i) {
		pio_sm_claim(cfg->pio, cfg->sm_tmds[i]);
		dvi_serialiser_program_init(
			cfg->pio,
			cfg->sm_tmds[i],
			offset,
			cfg->pins_tmds[i],
			DVI_SERIAL_DEBUG
		);
		dvi_configure_pad(cfg->pins_tmds[i], cfg->invert_diffpairs);
		dvi_configure_pad(cfg->pins_tmds[i] + 1, cfg->invert_diffpairs);
	}

	// Use a PWM slice to drive the pixel clock. Both GPIOs must be on the same
	// slice (lower-numbered GPIO must be even).
	assert(cfg->pins_clk % 2 == 0);
	uint slice = pwm_gpio_to_slice_num(cfg->pins_clk);
	// 5 cycles high, 5 low. Invert one channel so that we get complementary outputs.
	pwm_config pwm_cfg = pwm_get_default_config();
	pwm_config_set_output_polarity(&pwm_cfg, true, false);
	pwm_config_set_wrap(&pwm_cfg, 9);
	pwm_init(slice, &pwm_cfg, false);
	pwm_set_both_levels(slice, 5, 5);

	for (uint i = cfg->pins_clk; i <= cfg->pins_clk + 1; ++i) {
		gpio_set_function(i, GPIO_FUNC_PWM);
		dvi_configure_pad(i, cfg->invert_diffpairs);
	}
}

void dvi_serialiser_enable(struct dvi_serialiser_cfg *cfg, bool enable) {
	uint mask = 0;
	for (int i = 0; i < N_TMDS_LANES; ++i)
		mask |= 1u << (cfg->sm_tmds[i] + PIO_CTRL_SM_ENABLE_LSB);
	if (enable) {
		hw_set_bits(&cfg->pio->ctrl, mask);
		pwm_set_enabled(pwm_gpio_to_slice_num(cfg->pins_clk), true);
	}
	else {
		hw_clear_bits(&cfg->pio->ctrl, mask);
		pwm_set_enabled(pwm_gpio_to_slice_num(cfg->pins_clk), false);
	}
}
