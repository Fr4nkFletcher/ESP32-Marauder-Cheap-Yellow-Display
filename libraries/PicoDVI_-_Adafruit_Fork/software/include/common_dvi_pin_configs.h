#ifndef _COMMON_DVI_PIN_CONFIGS_H
#define _COMMON_DVI_PIN_CONFIGS_H

// This file defines the TMDS pair layouts on a handful of boards I have been
// developing on. It's not a particularly important file -- just saves some
// copy + paste.

#if defined(ARDUINO)
#include "../libdvi/dvi_serialiser.h"
#else
#include "dvi_serialiser.h"
#endif

#ifndef DVI_DEFAULT_SERIAL_CONFIG
#define DVI_DEFAULT_SERIAL_CONFIG pico_sock_cfg
#endif

#ifndef DVI_DEFAULT_PIO_INST
#define DVI_DEFAULT_PIO_INST pio0
#endif

// ----------------------------------------------------------------------------
// PicoDVI boards

// Legacy pin mapping for Rev A PicoDVI boards -- I think just Graham and I
// have these :)
static const struct dvi_serialiser_cfg picodvi_reva_dvi_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {24, 26, 28},
	.pins_clk = 22,
	.invert_diffpairs = true
};

// AMY-DVI board, for getting HDMI from the RP2350 FPGA development platform,
// again a cursed board that only a couple of people in the world possess:
static const struct dvi_serialiser_cfg amy_dvi_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {14, 16, 18},
	.pins_clk = 12,
	.invert_diffpairs = true
};


// The not-HDMI socket on Rev C PicoDVI boards
// (we don't talk about Rev B)
static const struct dvi_serialiser_cfg picodvi_dvi_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {10, 12, 14},
	.pins_clk = 8,
	.invert_diffpairs = true
};

// You can jam an adapter board into either of the PMOD sockets on a PicoDVI!
static const struct dvi_serialiser_cfg picodvi_pmod0_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {2, 4, 0},
	.pins_clk = 6,
	.invert_diffpairs = false
};

// ----------------------------------------------------------------------------
// Other boards

// The not-HDMI socket on SparkX HDMI carrier board with RP2040 MicroMod
// inserted.
static const struct dvi_serialiser_cfg micromod_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {18, 20, 22},
	.pins_clk = 16,
	.invert_diffpairs = true
};

// Pico DVI Sock (small hat on the bottom) which solders to the end of a Pico
static const struct dvi_serialiser_cfg pico_sock_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {12, 18, 16},
	.pins_clk = 14,
	.invert_diffpairs = false
};

// The HDMI socket on Pimoroni Pico Demo HDMI
// (we would talk about rev B if we had a rev B...)
static const struct dvi_serialiser_cfg pimoroni_demo_hdmi_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {8, 10, 12},
	.pins_clk = 6,
	.invert_diffpairs = true
};

// Not HDMI Featherwing
static const struct dvi_serialiser_cfg not_hdmi_featherwing_cfg = {
	.pio = pio0,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {11, 9, 7},
	.pins_clk = 24,
	.invert_diffpairs = true
};

// Adafruit Feather RP2040 DVI
static const struct dvi_serialiser_cfg adafruit_feather_dvi_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {18, 20, 22},
	.pins_clk = 16,
	.invert_diffpairs = true
};

static const struct dvi_serialiser_cfg adafruit_dvibell_cfg = {
  .pio = DVI_DEFAULT_PIO_INST,
  .sm_tmds = {0, 1, 2},
  .pins_tmds = {10, 8, 6},
  .pins_clk = 12,
  .invert_diffpairs = false,
};

// Adafruit 'Fruit Jam' computer board
static const struct dvi_serialiser_cfg adafruit_fruitjam_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {14, 16, 18},
	.pins_clk = 12,
	.invert_diffpairs = true,
};



// Waveshare RP2040-PiZero
static const struct dvi_serialiser_cfg waveshare_rp2040_pizero = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {26, 24, 22},
	.pins_clk = 28,
	.invert_diffpairs = false
};

// Olimex RP2040PC, RP2040-Pico-PC, Neo6502, 
static struct dvi_serialiser_cfg olimex_rp2040_cfg = {
    .pio = DVI_DEFAULT_PIO_INST,
    .sm_tmds = {0, 1, 2},
    .pins_tmds = {14, 18, 16},
    .pins_clk = 12,
    .invert_diffpairs = true
};

#endif
