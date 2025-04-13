#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

#include "sprite_asm_const.h"

#define POP2_OFFS (SIO_INTERP0_POP_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)

.syntax unified
.cpu cortex-m0plus
.thumb

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
	lsrs \rx, \rs, #ALPHA_SHIFT_16BPP
	bcc 1f
	strh \rs, [\rd, #\dstoffs]
1:
	lsrs \rx, \rs, #ALPHA_SHIFT_16BPP + 16
	bcc 1f
	lsrs \rs, #16
	strh \rs, [\rd, #\dstoffs + 2]
1:
.endm

.macro do_2px_16bpp rd rs dstoffs
	strh \rs, [\rd, #\dstoffs]
	lsrs \rs, #16
	strh \rs, [\rd, #\dstoffs + 2]
.endm

// interp1 has been set up to give the next x-ward pointer into the tilemap
// with each pop. This saves us having to remember the tilemap pointer and
// tilemap x size mask in core registers.

// r0: dst
// r1: tileset
// r2: x0 (start pos in tile space)
// r3: x1 (end pos in tile space, exclusive)

// Instantiated with alpha=1 and alpha=0 to get both variants of the loop.
// Linker garbage collection ensures we only keep the versions we use.

.macro tile16_16px_loop_alpha_or_nonalpha alpha
	push {r4-r7, lr}
	mov r4, r8
	mov r5, r9
	push {r4, r5}
	ldr r7, =(SIO_BASE + SIO_INTERP1_ACCUM0_OFFSET)

	// The main loop only handles whole tiles, so we may need to first copy
	// individual pixels to get tile-aligned. Skip this entirely if we are
	// already aligned, to avoid the extra interp pop.
	lsls r6, r2, #28
	beq 3f

	// Get pointer to tileset image
	ldr r4, [r7, #POP2_OFFS]
	ldrb r4, [r4]
	lsls r4, #9
	add r4, r1
	// Offset tile image pointer to align with x0
	lsls r5, r2, #28
	lsrs r5, #27
	add r4, r5
	// Fall through into copy loop
1:
	ldrh r5, [r4]
.if \alpha
	lsrs r6, r5, #ALPHA_SHIFT_16BPP
	bcc 2f
.endif
	strh r5, [r0]
2:
	adds r4, #2
	adds r0, #2
	adds r2, #1
	lsls r6, r2, #28
	bne 1b
3:
	// The next output pixel is aligned to the start of a tile. Set up main loop.

	// Tileset pointer is only needed for an add operand:
	mov r8, r1
	// dst limit pointer at end of all pixels:
	subs r3, r2
	lsls r4, r3, #1
	add r4, r0
	mov r9, r4
	// dst limit pointer at end of whole tiles:
	lsrs r4, r3, #4
	lsls r4, #5
	add r4, r0
	mov ip, r4

	// r0 is dst, r7 is interp base, r1-r6 are free for loop.
	b 3f
2:
	// Get next tilemap pointer
	ldr r1, [r7, #POP2_OFFS]
	// Get tile image pointer
	ldrb r1, [r1]
	lsls r1, #9
	add r1, r8

.if \alpha
	ldmia r1!, {r3-r6}
	do_2px_16bpp_alpha r0 r3 r2 0
	do_2px_16bpp_alpha r0 r4 r2 4
	do_2px_16bpp_alpha r0 r5 r2 8
	do_2px_16bpp_alpha r0 r6 r2 12
	ldmia r1!, {r3-r6}
	do_2px_16bpp_alpha r0 r3 r2 16
	do_2px_16bpp_alpha r0 r4 r2 20
	do_2px_16bpp_alpha r0 r5 r2 24
	do_2px_16bpp_alpha r0 r6 r2 28
.else
	ldmia r1!, {r3-r6}
	do_2px_16bpp r0 r3 0
	do_2px_16bpp r0 r4 4
	do_2px_16bpp r0 r5 8
	do_2px_16bpp r0 r6 12
	ldmia r1!, {r3-r6}
	do_2px_16bpp r0 r3 16
	do_2px_16bpp r0 r4 20
	do_2px_16bpp r0 r5 24
	do_2px_16bpp r0 r6 28
.endif
	adds r0, 32
3:
	cmp r0, ip
	blo 2b

	// Tidy up runt tile at end. Don't worry about extra interp pop.
	ldr r4, [r7, #POP2_OFFS]
	ldrb r4, [r4]
	lsls r4, #9
	add r4, r8
	b 3f
1:
	ldrh r5, [r4]
.if \alpha
	lsrs r6, r5, #ALPHA_SHIFT_16BPP
	bcc 2f
.endif
	strh r5, [r0]
2:
	adds r4, #2
	adds r0, #2
3:
	cmp r0, r9
	blo 1b

	pop {r4, r5}
	mov r8, r4
	mov r9, r5
	pop {r4-r7, pc}
.endm

decl_func tile16_16px_alpha_loop
	tile16_16px_loop_alpha_or_nonalpha 1

decl_func tile16_16px_loop
	tile16_16px_loop_alpha_or_nonalpha 0
