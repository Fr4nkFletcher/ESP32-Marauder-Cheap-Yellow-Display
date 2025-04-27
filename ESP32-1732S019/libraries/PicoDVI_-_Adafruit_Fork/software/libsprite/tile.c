#include "tile.h"

#include "pico.h" // for __not_in_flash
#include "hardware/interp.h"

#define __ram_func(foo) __not_in_flash(#foo) foo

static inline uint __attribute__((always_inline)) tile_log_size(tilesize_t size) {
	return 3 + (int)size;
};

static inline void setup_interp_tilemap_ptrs(interp_hw_t *interp, const uint8_t *row, uint x0, uint x_msb) {
	// Setup interpolator to add 1 to tile x, mask it with tile x mask, and
	// then add to tilemap row base. Since it's a preincrement, we walk the
	// initial x back by 1. This isn't a very exciting use of interpolators,
	// but it saves ~3 core registers for the pixel loops.
	interp_config c = interp_default_config();
	interp_config_set_mask(&c, 0, x_msb);
	interp_set_config(interp, 0, &c);
	interp->accum[0] = x0;
	interp->base[0] = 1;
	interp->ctrl[1] = 0;
	interp->base[2] = (uintptr_t)row;
}

void __ram_func(tile16)(uint16_t *scanbuf, const tilebg_t *bg, uint raster_y, uint raster_w) {
	uint size_x_mask = (1u << bg->log_size_x) - 1;
	uint size_y_mask = (1u << bg->log_size_y) - 1;
	// Find render start/end point in tile space
	// Note tx1 may be "past the end" -- that's fine, it's just used for limits
	uint tx0 = bg->xscroll & size_x_mask;
	uint tx1 = tx0 + raster_w;
	uint ty = (bg->yscroll + raster_y) & size_y_mask;

	const uint8_t *tilemap_row_ty = bg->tilemap + (ty >> tile_log_size(bg->tilesize)
		<< (bg->log_size_x - tile_log_size(bg->tilesize)));
	uint tile_x_at_tx0 = tx0 >> tile_log_size(bg->tilesize);
	uint tile_x_msb = bg->log_size_x - tile_log_size(bg->tilesize) - 1;

	// NOTE this clobbers interp1, currently this will cause issues if you try
	// to run tile code and certain TMDS encode loops on the same core. Could
	// be fixed by save/restore, at the cost of some performance.
	setup_interp_tilemap_ptrs(interp1_hw, tilemap_row_ty, tile_x_at_tx0, tile_x_msb);

	// Apply intra-tile y offset in advance, since this will be the same for
	// all pixels of all tiles we render in this call.
	uint tilesize = 1u << tile_log_size(bg->tilesize);
	const uint16_t *tileset_y_offs = (const uint16_t*)bg->tileset +
		(ty & (tilesize - 1)) * tilesize;

	tile16_loop_t loop = (tile16_loop_t)bg->fill_loop;
	loop(scanbuf, tileset_y_offs, tx0, tx1);
}
