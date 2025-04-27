#ifndef _TILE_FUNCS_H
#define _TILE_FUNCS_H

#include "pico/types.h"

typedef enum {
	TILESIZE_8 = 0,
	TILESIZE_16
} tilesize_t;

// Dubiously-erased function type for fill loops (cast out before calling):
typedef void (*tile_loop_t)(void *dst, const void *tileset, uint x0, uint x1);

// Rather than defining the pixel format, the tilebg object has a function
// pointer to a pixel fill loop appropriate for the format and tile size.
//
// The asm fill loops are specialised for format (as well as tile size), are
// pretty large, and live in RAM. If we dispatched those loops dynamically,
// the references to all the loops would prevent their link-time garbage
// collection, wasting RAM we would rather use for sprite/tile assets.
// Instead, the creator of the tilebg object explicitly adds references to
// the appropriate fill routine symbols when configuring the tilebgs.

typedef struct tilebg {
	uint16_t xscroll;
	uint16_t yscroll;
	const void *tileset;
	const uint8_t *tilemap;
	uint8_t log_size_x;
	uint8_t log_size_y;
	tilesize_t tilesize;
	tile_loop_t fill_loop;
} tilebg_t;

// ----------------------------------------------------------------------------
// Functions from tile.S

// Signature of fill loops of a given pixel size:
typedef void (*tile16_loop_t)(uint16_t *dst, const uint16_t *tileset, uint x0, uint x1);
typedef void (*tile8_loop_t)(uint8_t *dst, const uint8_t *tileset, uint x0, uint x1);

void tile16_16px_alpha_loop(uint16_t *dst, const uint16_t *tileset, uint x0, uint x1);
void tile16_16px_loop(uint16_t *dst, const uint16_t *tileset, uint x0, uint x1);

// ----------------------------------------------------------------------------
// Functions from tile.c

void tile16(uint16_t *scanbuf, const tilebg_t *bg, uint raster_y, uint raster_w);



#endif
