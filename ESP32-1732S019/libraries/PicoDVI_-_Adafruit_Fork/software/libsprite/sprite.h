#ifndef _SPRITE_H
#define _SPRITE_H

#include "pico/types.h"
#include "affine_transform.h"

// 3 words -- any bigger and we will need to pack the flags!
typedef struct sprite {
	int16_t x;
	int16_t y;
	const void *img;
	uint8_t log_size; // always square
	bool has_opacity_metadata;
	bool hflip;
	bool vflip;
} sprite_t;

// ----------------------------------------------------------------------------
// Functions from sprite.S

// Constant-colour span
void sprite_fill8(uint8_t *dst, uint8_t colour, uint len);
void sprite_fill16(uint16_t *dst, uint16_t colour, uint len);

// Block image transfers
void sprite_blit8(uint8_t *dst, const uint8_t *src, uint len);
void sprite_blit8_alpha(uint8_t *dst, const uint8_t *src, uint len);
void sprite_blit16(uint16_t *dst, const uint16_t *src, uint len);
void sprite_blit16_alpha(uint16_t *dst, const uint16_t *src, uint len);

// These are just inner loops, and require INTERP0 to be configured before calling:
void sprite_ablit8_loop(uint8_t *dst, uint len);
void sprite_ablit8_alpha_loop(uint8_t *dst, uint len);
void sprite_ablit16_loop(uint16_t *dst, uint len);
void sprite_ablit16_alpha_loop(uint16_t *dst, uint len);

// ----------------------------------------------------------------------------
// Functions from sprite.c

// Render the intersection of a sprite with the current scanline:
void sprite_sprite8(uint8_t *scanbuf, const sprite_t *sp, uint raster_y, uint raster_w);
void sprite_sprite16(uint16_t *scanbuf, const sprite_t *sp, uint raster_y, uint raster_w);

// As above, but apply an affine transform on sprite texture lookups (SLOW, even with interpolator)
void sprite_asprite8(uint8_t *scanbuf, const sprite_t *sp, const affine_transform_t atrans, uint raster_y, uint raster_w);
void sprite_asprite16(uint16_t *scanbuf, const sprite_t *sp, const affine_transform_t atrans, uint raster_y, uint raster_w);

#endif
