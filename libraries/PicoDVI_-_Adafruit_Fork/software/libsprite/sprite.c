#include "sprite.h"
#include "affine_transform.h"

#include "pico.h" // for __not_in_flash
#include "hardware/interp.h"

// Note some of the sprite routines are quite large (unrolled), so trying to
// keep everything in separate sections so the linker can garbage collect
// unused sprite code. In particular we usually need 8bpp xor 16bpp functions!
#define __ram_func(foo) __not_in_flash(#foo) foo

typedef struct {
	int tex_offs_x;
	int tex_offs_y;
	int size_x;
} intersect_t;

// Always-inline else the compiler does rash things like passing structs in memory
static inline intersect_t _get_sprite_intersect(const sprite_t *sp, uint raster_y, uint raster_w) {
	intersect_t isct = {0};
	isct.tex_offs_y = (int)raster_y - sp->y;
	int size = 1u << sp->log_size;
	uint upper_mask = -size;
	if ((uint)isct.tex_offs_y & upper_mask)
		return isct;
	int x_start_clipped = MAX(0, sp->x);
	isct.tex_offs_x = x_start_clipped - sp->x;
	isct.size_x = MIN(sp->x + size, (int)raster_w) - x_start_clipped;
	return isct;
}

// Sprites may have an array of metadata on the end. One word per line, encodes first opaque pixel, last opaque pixel, and whether the span in between is solid. This allows fewer 
static inline intersect_t _intersect_with_metadata(intersect_t isct, uint32_t meta) {
	int span_end = meta & 0xffff;
	int span_start = (meta >> 16) & 0x7fff;
	int isct_new_start = MAX(isct.tex_offs_x, span_start);
	int isct_new_end = MIN(isct.tex_offs_x + isct.size_x, span_end);
	isct.tex_offs_x = isct_new_start;
	isct.size_x = isct_new_end - isct_new_start;
	return isct;
}

void __ram_func(sprite_sprite8)(uint8_t *scanbuf, const sprite_t *sp, uint raster_y, uint raster_w) {
	int size = 1u << sp->log_size;
	intersect_t isct = _get_sprite_intersect(sp, raster_y, raster_w);
	if (isct.size_x <= 0)
		return;
	if (sp->vflip)
		isct.tex_offs_y = size - 1 - isct.tex_offs_y;
	const uint8_t *img = sp->img;
	if (sp->has_opacity_metadata) {
		// Metadata is one word per row, concatenated to end of pixel data
		uint32_t meta = ((uint32_t*)(sp->img + size * size * sizeof(uint8_t)))[isct.tex_offs_y];
		isct = _intersect_with_metadata(isct, meta);
		if (isct.size_x <= 0)
			return;
		bool span_continuous = !!(meta & (1u << 31));
		if (span_continuous) {
			// Non-alpha blit is ~50% faster
			sprite_blit8(scanbuf + sp->x + isct.tex_offs_x, img + isct.tex_offs_x + isct.tex_offs_y * size, isct.size_x);
		}
		else {
			sprite_blit8_alpha(scanbuf + sp->x + isct.tex_offs_x, img + isct.tex_offs_x + isct.tex_offs_y * size, isct.size_x);
		}
	}
	else {
		sprite_blit8_alpha(scanbuf + sp->x + isct.tex_offs_x, img + isct.tex_offs_x + isct.tex_offs_y * size, isct.size_x);
	}
}

void __ram_func(sprite_sprite16)(uint16_t *scanbuf, const sprite_t *sp, uint raster_y, uint raster_w) {
	int size = 1u << sp->log_size;
	intersect_t isct = _get_sprite_intersect(sp, raster_y, raster_w);
	if (isct.size_x <= 0)
		return;
	if (sp->vflip)
		isct.tex_offs_y = size - 1 - isct.tex_offs_y;
	const uint16_t *img = sp->img;
	if (sp->has_opacity_metadata) {
		uint32_t meta = ((uint32_t*)(sp->img + size * size * sizeof(uint16_t)))[isct.tex_offs_y];
		isct = _intersect_with_metadata(isct, meta);
		if (isct.size_x <= 0)
			return;
		bool span_continuous = !!(meta & (1u << 31));
		if (span_continuous)
			sprite_blit16(scanbuf + sp->x + isct.tex_offs_x, img + isct.tex_offs_x + isct.tex_offs_y * size, isct.size_x);
		else
			sprite_blit16_alpha(scanbuf + sp->x + isct.tex_offs_x, img + isct.tex_offs_x + isct.tex_offs_y * size, isct.size_x);
	}
	else {
		sprite_blit16_alpha(scanbuf + MAX(0, sp->x), img + isct.tex_offs_x + isct.tex_offs_y * size, isct.size_x);
	}
}

// We're defining the affine transform as:
//
// [u]   [ a00 a01 b0 ]   [x]   [a00 * x + a01 * y + b0]
// [v] = [ a10 a11 b1 ] * [y] = [a10 * x + a11 * y + b1]
// [1]   [ 0   0   1  ]   [1]   [           1          ]
//
// We represent this in memory as {a00, a01, b0, a10, a11, b1} (all int32_t)
// i.e. the non-constant parts in row-major order

// Set up an interpolator to follow a straight line through u,v space
static inline __attribute__((always_inline)) void _setup_interp_affine(interp_hw_t *interp, intersect_t isct, const affine_transform_t atrans) {
	// Calculate the u,v coord of the first sample. Note that we are iterating
	// *backward* along the raster span because this is faster (yes)
	int32_t x0 =
		mul_fp1616(atrans[0], (isct.tex_offs_x + isct.size_x) * AF_ONE) +
		mul_fp1616(atrans[1], isct.tex_offs_y * AF_ONE) +
		atrans[2];
	int32_t y0 =
		mul_fp1616(atrans[3], (isct.tex_offs_x + isct.size_x) * AF_ONE) +
		mul_fp1616(atrans[4], isct.tex_offs_y * AF_ONE) +
		atrans[5];
	interp->accum[0] = x0;
	interp->accum[1] = y0;
	interp->base[0] = -atrans[0]; // -a00, since x decrements by 1 with each coord
	interp->base[1] = -atrans[3]; // -a10
}

// Set up an interpolator to generate pixel lookup addresses from fp1616
// numbers in accum1, accum0 based on the parameters of sprite sp and the size
// of the individual pixels
static inline __attribute__((always_inline)) void _setup_interp_pix_coordgen(interp_hw_t *interp, const sprite_t *sp, uint pixel_shift) {
	// Concatenate from accum0[31:16] and accum1[31:16] as many LSBs as required
	// to index the sprite texture in both directions. Reading from POP_FULL will
	// yields these bits, added to sp->img, and this will also trigger BASE0 and
	// BASE1 to be directly added (thanks to CTRL_ADD_RAW) to the accumulators,
	// which generates the u,v coordinate for the *next* read.
	assert(sp->log_size + pixel_shift <= 16);

	interp_config c0 = interp_default_config();
	interp_config_set_add_raw(&c0, true);
	interp_config_set_shift(&c0, 16 - pixel_shift);
	interp_config_set_mask(&c0, pixel_shift, pixel_shift + sp->log_size - 1);
	interp_set_config(interp, 0, &c0);

	interp_config c1 = interp_default_config();
	interp_config_set_add_raw(&c1, true);
	interp_config_set_shift(&c1, 16 - sp->log_size - pixel_shift);
	interp_config_set_mask(&c1, pixel_shift + sp->log_size, pixel_shift + 2 * sp->log_size - 1);
	interp_set_config(interp, 1, &c1);

	interp->base[2] = (uint32_t)sp->img;
}

// Note we do NOT save/restore the interpolator!
void __ram_func(sprite_asprite8)(uint8_t *scanbuf, const sprite_t *sp, const affine_transform_t atrans, uint raster_y, uint raster_w) {
	intersect_t isct = _get_sprite_intersect(sp, raster_y, raster_w);
	if (isct.size_x <= 0)
		return;
	interp_hw_t *interp = interp0_hw;
	_setup_interp_affine(interp, isct, atrans);
	_setup_interp_pix_coordgen(interp, sp, 0);
	// Now every read from POP_FULL will give us a new MODE7 lookup pointer for sp->img :)
	sprite_ablit8_alpha_loop(scanbuf + MAX(0, sp->x), isct.size_x);
}

void __ram_func(sprite_asprite16)(uint16_t *scanbuf, const sprite_t *sp, const affine_transform_t atrans, uint raster_y, uint raster_w) {
	intersect_t isct = _get_sprite_intersect(sp, raster_y, raster_w);
	if (isct.size_x <= 0)
		return;
	interp_hw_t *interp = interp0_hw;
	_setup_interp_affine(interp, isct, atrans);
	_setup_interp_pix_coordgen(interp, sp, 1);
	sprite_ablit16_alpha_loop(scanbuf + MAX(0, sp->x), isct.size_x);
}
