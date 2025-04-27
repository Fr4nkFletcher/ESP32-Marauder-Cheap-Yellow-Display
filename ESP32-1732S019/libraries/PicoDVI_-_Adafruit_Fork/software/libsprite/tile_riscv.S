#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

#include "sprite_asm_const.h"

#define POP2_OFFS (SIO_INTERP0_POP_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)

// ----------------------------------------------------------------------------
// Tile layout
//
// Some terms:
// Tileset: 1D array of tile images, concatenated image-after-image
// Tilemap: 2D array of tileset indices
//
// Each tile image in a tileset is the same size. Tiles are square, either 8 x
// 8 px or 16 x 16 px. This makes it easy to find the start of a tile image
// given the tileset base pointer and a tile index (add + shift).
//
// Tilemaps are 8 bits per tile, always.
//
// One advantage of this layout is that y coordinates can be handled outside
// of the loops in this file, which are all scanline-oriented, by offsetting
// the tileset and tilemap pointers passed in. These routines only care about
// x. The tileset pointer is offset by y modulo tile height, and the tilemap
// pointer is offset by y divided by tile height, modulo tileset height in
// tiles.

// Tileset: 16px tiles, 16bpp, with 1-bit alpha.
// Tilemap: 8 bit indices.

.macro do_2px_16bpp_alpha rd rs rx dstoffs
.option push
.option norvc
	// TODO we could save a shift here by making alpha the MSB (not worth it
	// on Arm due to lack of sign-extension or flag update on loads)
	slli \rx, \rs, 32 - ALPHA_SHIFT_16BPP
	bgez \rx, 1f
	sh \rs, \dstoffs(\rd)
1:
	slli \rx, \rs, 16 - ALPHA_SHIFT_16BPP
	bgez \rx, 1f
	srli \rs, \rs, 16
	sh \rs, \dstoffs+2(\rd)
1:
.option pop
.endm

.macro do_2px_16bpp rd rs dstoffs
	sh \rs, \dstoffs(\rd)
	srli \rs, \rs, 16
	sh \rs, \dstoffs+2(\rd)
.endm

// interp1 has been set up to give the next x-ward pointer into the tilemap
// with each pop. This saves us having to remember the tilemap pointer and
// tilemap x size mask in core registers.

// a0: dst
// a1: tileset
// a2: x0 (start pos in tile space)
// a3: x1 (end pos in tile space, exclusive)

// Instantiated with alpha=1 and alpha=0 to get both variants of the loop.
// Linker garbage collection ensures we only keep the versions we use.

.macro tile16_16px_loop_alpha_or_nonalpha alpha
	li a7, SIO_BASE + SIO_INTERP1_ACCUM0_OFFSET

	// The main loop only handles whole tiles, so we may need to first copy
	// individual pixels to get tile-aligned. Skip this entirely if we are
	// already aligned, to avoid the extra interp pop.
	andi a5, a2, 0xf
	beqz a5, 3f

	// Get pointer to tileset image
	lw a4, POP2_OFFS(a7)
	lbu a4, (a4)   // dep stall
	slli a4, a4, 9 // 16 px wide * 16 px high * 2 bytes/px
	add a4, a4, a1
	// Offset tile image pointer to align with x0
	sh1add a4, a5, a4
	// Fall through into copy loop
1:
	lhu a5, (a4)
	addi a4, a4, 2 // hoisted to fill load dependency slot
.if \alpha
	slli a6, a5, 32 - ALPHA_SHIFT_16BPP
	bgez a6, 2f
.endif
	sh a5, (a0)
2:
	addi a0, a0, 2
	addi a2, a2, 1
	// Skip out if we have already reached end of span:
	bgeu a2, a3, 3f
	// Loop if we are not yet aligned: (TODO these checks could be merged)
	andi a6, a2, 0xf
	bnez a6, 1b
3:
	// The next output pixel is aligned to the start of a tile. Set up main loop.

	// Tileset pointer is only needed occasionally, so free up a1 for better
	// code density:
	mv t0, a1
	// t1: dst limit pointer at end of all pixels:
	sub a3, a3, a2
	sh1add t1, a3, a0
	// a5: dst limit pointer at end of whole tiles:
	andi a4, a3, ~0xf
	sh1add a5, a4, a0

	// a0 is dst, a7 is interp base, a1-a4 are trashed by loop, a5 is dst limit.
	// Early skip for case of 0 whole tiles:
	bgeu a0, a5, 3f
2:
	// Get next tilemap pointer
	lw a1, POP2_OFFS(a7)
	// Get tile image pointer
	lbu a1, (a1) // dep stall
	slli a1, a1, 9
	add a1, a1, t0

.if \alpha
	lw a3,  0(a1)
	lw a4,  4(a1)
	do_2px_16bpp_alpha a0 a3 a2 0
	do_2px_16bpp_alpha a0 a4 a2 4
	lw a3,  8(a1)
	lw a4, 12(a1)
	do_2px_16bpp_alpha a0 a3 a2 8
	do_2px_16bpp_alpha a0 a4 a2 12
	lw a3, 16(a1)
	lw a4, 20(a1)
	do_2px_16bpp_alpha a0 a3 a2 16
	do_2px_16bpp_alpha a0 a4 a2 20
	lw a3, 24(a1)
	lw a4, 28(a1)
	do_2px_16bpp_alpha a0 a3 a2 24
	do_2px_16bpp_alpha a0 a4 a2 28
.else
	lw a3,  0(a1)
	lw a4,  4(a1)
	do_2px_16bpp a0 a3 0
	do_2px_16bpp a0 a4 4
	lw a3,  8(a1)
	lw a4, 12(a1)
	do_2px_16bpp a0 a3 8
	do_2px_16bpp a0 a4 12
	lw a3, 16(a1)
	lw a4, 20(a1)
	do_2px_16bpp a0 a3 16
	do_2px_16bpp a0 a4 20
	lw a3, 24(a1)
	lw a4, 28(a1)
	do_2px_16bpp a0 a3 24
	do_2px_16bpp a0 a4 28
.endif
	addi a0, a0, 32
	bltu a0, a5, 2b
3:

	// Skip ahead if there are no spare pixels to tidy up	
	bgeu a0, t1, 3f
	// Copy <1 tile's worth of loose pixels
	lw a4, POP2_OFFS(a7)
	lbu a4, (a4) // dep stall
	slli a4, a4, 9
	add a4, a4, t0
1:
	lh a5, (a4)
	addi a4, a4, 2
.if \alpha
	slli a6, a5, 32 - ALPHA_SHIFT_16BPP
	bgez a6, 2f
.endif
	sh a5, (a0)
2:
	addi a0, a0, 2
	bltu a0, t1, 1b
3:
	ret
.endm

decl_func tile16_16px_alpha_loop
	tile16_16px_loop_alpha_or_nonalpha 1

decl_func tile16_16px_loop
	tile16_16px_loop_alpha_or_nonalpha 0
