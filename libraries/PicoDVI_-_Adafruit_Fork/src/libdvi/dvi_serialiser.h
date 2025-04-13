#ifndef _DVI_SERIALISER_H
#define _DVI_SERIALISER_H

#include "hardware/pio.h"
#include "dvi_config_defs.h"

#define N_TMDS_LANES 3

struct dvi_serialiser_cfg {
	PIO pio;
	uint sm_tmds[N_TMDS_LANES];
	uint pins_tmds[N_TMDS_LANES];
	uint pins_clk;
	bool invert_diffpairs;
	uint prog_offs;
};

void dvi_serialiser_init(struct dvi_serialiser_cfg *cfg);
void dvi_serialiser_enable(struct dvi_serialiser_cfg *cfg, bool enable);
uint32_t dvi_single_to_diff(uint32_t in);

#endif
