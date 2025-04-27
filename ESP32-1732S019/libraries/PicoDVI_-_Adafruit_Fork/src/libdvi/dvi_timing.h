#ifndef _DVI_TIMING_H
#define _DVI_TIMING_H

#include "hardware/dma.h"
#include "pico/util/queue.h"

#include "dvi.h"

struct dvi_timing {
	bool h_sync_polarity;
	uint h_front_porch;
	uint h_sync_width;
	uint h_back_porch;
	uint h_active_pixels;

	bool v_sync_polarity;
	uint v_front_porch;
	uint v_sync_width;
	uint v_back_porch;
	uint v_active_lines;

	uint bit_clk_khz;
};

enum dvi_line_state {
	DVI_STATE_FRONT_PORCH = 0,
	DVI_STATE_SYNC,
	DVI_STATE_BACK_PORCH,
	DVI_STATE_ACTIVE,
	DVI_STATE_COUNT
};

struct dvi_timing_state {
	uint v_ctr;
	enum dvi_line_state v_state;
};

// This should map directly to DMA register layout, but more convenient types
// (also this really shouldn't be here... we don't have a dma_cb in the SDK
// because there are many valid formats due to aliases)
typedef struct dma_cb {
	const void *read_addr;
	void *write_addr;
	uint32_t transfer_count;
	dma_channel_config c;
} dma_cb_t;

static_assert(sizeof(dma_cb_t) == 4 * sizeof(uint32_t), "bad dma layout");
static_assert(__builtin_offsetof(dma_cb_t, c.ctrl) == __builtin_offsetof(dma_channel_hw_t, ctrl_trig), "bad dma layout");

#define DVI_SYNC_LANE_CHUNKS DVI_STATE_COUNT
#define DVI_NOSYNC_LANE_CHUNKS 2

struct dvi_scanline_dma_list {
	dma_cb_t l0[DVI_SYNC_LANE_CHUNKS];
	dma_cb_t l1[DVI_NOSYNC_LANE_CHUNKS];
	dma_cb_t l2[DVI_NOSYNC_LANE_CHUNKS];
};

static inline dma_cb_t* dvi_lane_from_list(struct dvi_scanline_dma_list *l, int i) {
	return i == 0 ? l->l0 : i == 1 ? l->l1 : l->l2;
}

// Each TMDS lane uses one DMA channel to transfer data to a PIO state
// machine, and another channel to load control blocks into this channel.
struct dvi_lane_dma_cfg {
	uint chan_ctrl;
	uint chan_data;
	void *tx_fifo;
	uint dreq;
};

// Note these are already converted to pseudo-differential representation
extern const uint32_t dvi_ctrl_syms[4];

extern const struct dvi_timing dvi_timing_640x480p_60hz;
extern const struct dvi_timing dvi_timing_800x480p_60hz;
extern const struct dvi_timing dvi_timing_800x480p_30hz;
extern const struct dvi_timing dvi_timing_800x600p_60hz;
extern const struct dvi_timing dvi_timing_960x540p_60hz;
extern const struct dvi_timing dvi_timing_1280x720p_30hz;

extern const struct dvi_timing dvi_timing_800x600p_reduced_60hz;
extern const struct dvi_timing dvi_timing_1280x720p_reduced_30hz;

void dvi_timing_state_init(struct dvi_timing_state *t);

void dvi_timing_state_advance(const struct dvi_timing *t, struct dvi_timing_state *s);

void dvi_scanline_dma_list_init(struct dvi_scanline_dma_list *dma_list);

void dvi_setup_scanline_for_vblank(const struct dvi_timing *t, const struct dvi_lane_dma_cfg dma_cfg[],
		bool vsync_asserted, struct dvi_scanline_dma_list *l);

void dvi_setup_scanline_for_active(const struct dvi_timing *t, const struct dvi_lane_dma_cfg dma_cfg[],
		uint32_t *tmdsbuf, struct dvi_scanline_dma_list *l);

void dvi_update_scanline_data_dma(const struct dvi_timing *t, const uint32_t *tmdsbuf, struct dvi_scanline_dma_list *l);

#endif
