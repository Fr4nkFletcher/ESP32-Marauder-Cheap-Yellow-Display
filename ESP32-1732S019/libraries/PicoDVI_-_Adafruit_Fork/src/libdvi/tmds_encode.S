#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#include "dvi_config_defs.h"

// Offsets suitable for ldr/str (must be <= 0x7c):
#define ACCUM0_OFFS     (SIO_INTERP0_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_OFFS     (SIO_INTERP0_ACCUM1_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_ADD_OFFS (SIO_INTERP0_ACCUM1_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK0_OFFS      (SIO_INTERP0_PEEK_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK1_OFFS      (SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK2_OFFS      (SIO_INTERP0_PEEK_FULL_OFFSET  - SIO_INTERP0_ACCUM0_OFFSET)
#define INTERP1         (SIO_INTERP1_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
// Note the entirety of INTERP0 and INTERP1 fits inside this 5-bit
// word-addressed space... almost as though it were intentional! :)

.syntax unified
.cpu cortex-m0plus
.thumb

.macro decl_func_x name
.section .scratch_x.\name, "ax"
.global \name
.type \name,%function
.thumb_func
\name:
.endm

.macro decl_func_y name
.section .scratch_y.\name, "ax"
.global \name
.type \name,%function
.thumb_func
\name:
.endm

#define decl_func decl_func_x

// ----------------------------------------------------------------------------
// Pixel-doubling encoders for RGB

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Input size (pixels)

.macro do_channel_16bpp r_ibase r_inout0 r_out1
	str \r_inout0, [\r_ibase, #ACCUM0_OFFS]
	ldr \r_inout0, [\r_ibase, #PEEK0_OFFS]
	ldr \r_inout0, [\r_inout0]
	ldr \r_out1, [\r_ibase, #PEEK1_OFFS]
	ldr \r_out1, [\r_out1]
.endm

decl_func tmds_encode_loop_16bpp
	push {r4, r5, r6, r7, lr}
	lsls r2, #2
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	b 2f
.align 2
1:
.rept TMDS_ENCODE_UNROLL
	ldmia r0!, {r4, r6}
	do_channel_16bpp r2, r4, r5
	do_channel_16bpp r2, r6, r7
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	bne 1b
	pop {r4, r5, r6, r7, pc}

// Same as above, but scale data to make up for lack of left shift
// in interpolator (costs 1 cycle per 2 pixels)
//
// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Input size (pixels)
// r3: Left shift amount

decl_func tmds_encode_loop_16bpp_leftshift
	push {r4, r5, r6, r7, lr}
	lsls r2, #2
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	b 2f
.align 2
1:
.rept TMDS_ENCODE_UNROLL
	ldmia r0!, {r4, r6}
	lsls r4, r3
	do_channel_16bpp r2, r4, r5
	lsls r6, r3
	do_channel_16bpp r2, r6, r7
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	bne 1b
	pop {r4, r5, r6, r7, pc}

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Input size (pixels)

decl_func tmds_encode_loop_8bpp
	push {r4, r5, r6, r7, lr}
	lsls r2, #2
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	b 2f
.align 2
1:
.rept TMDS_ENCODE_UNROLL
	ldmia  r0!, {r4}
	str r4, [r2, #ACCUM0_OFFS + INTERP1]
	str r4, [r2, #ACCUM0_OFFS]
	ldr r4, [r2, #PEEK0_OFFS]
	ldr r4, [r4]
	ldr r5, [r2, #PEEK1_OFFS]
	ldr r5, [r5]
	ldr r6, [r2, #PEEK0_OFFS + INTERP1]
	ldr r6, [r6]
	ldr r7, [r2, #PEEK1_OFFS + INTERP1]
	ldr r7, [r7]
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	bne 1b
	pop {r4, r5, r6, r7, pc}

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Input size (pixels)
// r3: Left shift amount
//
// Note that only the data written to interp0 (pixel 0, 1) is leftshifted, not
// the data written to interp1 (pixel 2, 3). Otherwise we always lose MSBs, as
// the LUT offset MSB is at bit 8, so pixel 0 always requires some left shift,
// since its channel MSBs are no greater than 7.

decl_func tmds_encode_loop_8bpp_leftshift
	push {r4, r5, r6, r7, lr}
	lsls r2, #3
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	b 2f
.align 2
1:
.rept TMDS_ENCODE_UNROLL
	ldmia  r0!, {r4}
	str r4, [r2, #ACCUM0_OFFS + INTERP1]
	lsls r4, r3
	str r4, [r2, #ACCUM0_OFFS]
	ldr r4, [r2, #PEEK0_OFFS]
	ldr r4, [r4]
	ldr r5, [r2, #PEEK1_OFFS]
	ldr r5, [r5]
	ldr r6, [r2, #PEEK0_OFFS + INTERP1]
	ldr r6, [r6]
	ldr r7, [r2, #PEEK1_OFFS + INTERP1]
	ldr r7, [r7]
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	bne 1b
	pop {r4, r5, r6, r7, pc}

// ----------------------------------------------------------------------------
// Fast 1bpp black/white encoder (full res)

// Taking the encoder from DVI spec, with initial balance 0:
// 
// - Encoding either 0x00 or 0xff will produce a running balance of -8, with
//   output symbol of 0x100 or 0x200
// 
// - Subsequently encoding either 0x01 or 0xfe will return the balance to 0, with
//  output symbol of 0x1ff or 0x2ff
// 
// So we can do 1bpp encode with a lookup of x coordinate LSB, and input
// colour bit. If we process pixels in even-sized blocks, only the colour
// lookup is needed.

// Encode 8 pixels @ 1bpp (using two table lookups)
// r3 contains lookup mask (preshifted)
// r8 contains pointer to encode table
// 2.125 cyc/pix
.macro tmds_encode_1bpp_body shift_instr0 shamt0 shift_instr1 shamt1
	\shift_instr0 r4, r2, #\shamt0
	ands r4, r3
	add r4, r8
	ldmia r4, {r4, r5}
	\shift_instr1 r6, r2, #\shamt1
	ands r6, r3
	add r6, r8
	ldmia r6, {r6, r7}
	stmia r1!, {r4, r5, r6, r7}
.endm

// r0: input buffer (word-aligned)
// r1: output buffer (word-aligned)
// r2: output pixel count
decl_func tmds_encode_1bpp
	push {r4-r7, lr}
	mov r7, r8
	push {r7}
	lsls r2, #1
	add r2, r1
	mov ip, r2
	adr r4, tmds_1bpp_table
	mov r8, r4
	// Mask: 4 bit index, 8 bytes per entry
	movs r3, #0x78
	b 2f
1:
	ldmia r0!, {r2}
#if !DVI_1BPP_BIT_REVERSE
	tmds_encode_1bpp_body lsls 3  lsrs 1
	tmds_encode_1bpp_body lsrs 5  lsrs 9
	tmds_encode_1bpp_body lsrs 13 lsrs 17
	tmds_encode_1bpp_body lsrs 21 lsrs 25
#else
	tmds_encode_1bpp_body lsrs 1   lsls 3
	tmds_encode_1bpp_body lsrs 9   lsrs 5
	tmds_encode_1bpp_body lsrs 17  lsrs 13
	tmds_encode_1bpp_body lsrs 25  lsrs 21
#endif
2:
	cmp r1, ip
	blo 1b

	pop {r7}
	mov r8, r7
	pop {r4-r7, pc}

.align 2
tmds_1bpp_table:
#if !DVI_1BPP_BIT_REVERSE
	.word 0x7fd00, 0x7fd00  // 0000
	.word 0x7fe00, 0x7fd00  // 0001
	.word 0xbfd00, 0x7fd00  // 0010
	.word 0xbfe00, 0x7fd00  // 0011
	.word 0x7fd00, 0x7fe00  // 0100
	.word 0x7fe00, 0x7fe00  // 0101
	.word 0xbfd00, 0x7fe00  // 0110
	.word 0xbfe00, 0x7fe00  // 0111
	.word 0x7fd00, 0xbfd00  // 1000
	.word 0x7fe00, 0xbfd00  // 1001
	.word 0xbfd00, 0xbfd00  // 1010
	.word 0xbfe00, 0xbfd00  // 1011
	.word 0x7fd00, 0xbfe00  // 1100
	.word 0x7fe00, 0xbfe00  // 1101
	.word 0xbfd00, 0xbfe00  // 1110
	.word 0xbfe00, 0xbfe00  // 1111
#else
	.word 0x7fd00, 0x7fd00  // 0000
	.word 0x7fd00, 0xbfd00  // 1000
	.word 0x7fd00, 0x7fe00  // 0100
	.word 0x7fd00, 0xbfe00  // 1100
	.word 0xbfd00, 0x7fd00  // 0010
	.word 0xbfd00, 0xbfd00  // 1010
	.word 0xbfd00, 0x7fe00  // 0110
	.word 0xbfd00, 0xbfe00  // 1110
	.word 0x7fe00, 0x7fd00  // 0001
	.word 0x7fe00, 0xbfd00  // 1001
	.word 0x7fe00, 0x7fe00  // 0101
	.word 0x7fe00, 0xbfe00  // 1101
	.word 0xbfe00, 0x7fd00  // 0011
	.word 0xbfe00, 0xbfd00  // 1011
	.word 0xbfe00, 0x7fe00  // 0111
	.word 0xbfe00, 0xbfe00  // 1111
#endif


// ----------------------------------------------------------------------------
// Full-resolution 2bpp encode (for 2bpp grayscale, or bitplaned RGB222)

// Even-x-position pixels are encoded as symbols with imbalance -4, and odd
// pixels with +4, so that we can mix-and-match our even/odd codewords and
// always get a properly balanced sequence:
//
// level 0: (05 -> 103), then (04 -> 1fc)  (decimal 5, 4)
// level 1: (50 -> 130), then (51 -> 1cf)  (decimal 80, 81)
// level 2: (af -> 230), then (ae -> 2cf)  (decimal 175, 174)
// level 3: (fa -> 203), then (fb -> 2fc)  (decimal 250, 251)
//
// These correspond to roughly 255 times (0, 1/3, 2/3, 1).
//
// Alternatively we could use symbols with 0 balance, which results in lower
// contrast but avoids the LSB bobble:
//
// level 0: (10 -> 1f0) always
// level 1: (5a -> 263) always
// level 2: (a5 -> 163) always
// level 3: (ef -> 2f0) always

// Table base pointer in r0. Input pixels in r2.
.macro encode_2bpp_body shift_instr shamt rd
	\shift_instr \rd, r2, #\shamt
	ands \rd, r3
	ldr \rd, [r0, \rd]
.endm

// r0: input buffer (word-aligned)
// r1: output buffer (word-aligned)
// r2: output pixel count
decl_func tmds_encode_2bpp
	push {r4-r7, lr}
	mov r7, r8
	push {r7}
	mov r8, r0
	adr r0, tmds_2bpp_table
	// Mask: 4-bit index into 4-byte entries.
	movs r3, #0x3c
	// Limit pointer: 1 word per 2 pixels
	lsls r2, #1
	add r2, r1
	mov ip, r2
	b 2f
1:
	mov r4, r8
	ldmia r4!, {r2}
	mov r8, r4
	encode_2bpp_body lsls 2  r4
	encode_2bpp_body lsrs 2  r5
	encode_2bpp_body lsrs 6  r6
	encode_2bpp_body lsrs 10 r7
	stmia r1!, {r4-r7}
	encode_2bpp_body lsrs 14 r4
	encode_2bpp_body lsrs 18 r5
	encode_2bpp_body lsrs 22 r6
	encode_2bpp_body lsrs 26 r7
	stmia r1!, {r4-r7}
2:
	cmp r1, ip
	blo 1b
	pop {r7}
	mov r8, r7
	pop {r4-r7, pc}

.align 2
tmds_2bpp_table:
	.word 0x7f103 // 00, 00
	.word 0x7f130 // 01, 00
	.word 0x7f230 // 10, 00
	.word 0x7f203 // 11, 00
	.word 0x73d03 // 00, 01
	.word 0x73d30 // 01, 01
	.word 0x73e30 // 10, 01
	.word 0x73e03 // 11, 01
	.word 0xb3d03 // 00, 10
	.word 0xb3d30 // 01, 10
	.word 0xb3e30 // 10, 10
	.word 0xb3e03 // 11, 10
	.word 0xbf103 // 00, 11
	.word 0xbf130 // 01, 11
	.word 0xbf230 // 10, 11
	.word 0xbf203 // 11, 11

// ----------------------------------------------------------------------------
// Full-resolution RGB encode (not very practical)

// Non-doubled TMDS encode. 8.333 cycles per pixel, no exceptions. (This is
// taking horizontal blanking (at VGA) and dual core into account, and
// assuming the 3 channels are encoded individually.)
//
// Here is an idea
// Have a table with a 7 bit lookup. The lookup is the 6 colour data bits (in
// ACCUM0), concatenated with the sign bit of our running disparity (from
// ACCUM1). Each table entry is a 20-bit TMDS symbol (pseudodifferential),
// with the symbol's disparity stored left-justified in the upper 12 bits, as
// e.g. a 6 bit signed integer.
//
// - Load pixel data.                        cyc: 0.75 (ldmia 2 words, every 4 pixels)
// - Write pixel to ACCUM0.                  cyc: 1
// - Read address from PEEK2.                cyc: 1
// - Load encoded pixel from address.        cyc: 2
// - Write disparity data to ACCUM1_ADD      cyc: 1
// - Write encoded data to output buffer.    cyc: 1.25 (stmia 4 words, every 4 pixels)
//
// With decent register allocation we may be able to load 4 pixels at
// once (2 words), and write 4 at once (4 words). This gives 7 cyc/pix.
//
// One issue is that the TMDS data in the bottom of ACCUM1 will eventually
// overflow and affect the running disparity, but with 16 zeroes in between,
// this would take much longer than one scanline, so everything is fine if
// we clear the accumulator at the start of the scanline.
//
// Note that we need to use two interpolators to get the bits from both pixels
// -- we are not outputting a single DC-balanced stream, but rather two
// interleaved streams which are each DC-balanced. This is fine electrically,
// but our output here will *NOT* match the TMDS encoder given in the DVI
// spec.

// You can define TMDS_FULLRES_NO_DC_BALANCE to disable the running balance
// feedback. With the feedback enabled (default), the output is DC balanced,
// but there are just barely enough CPU cycles to do all the encode, so it's
// essentially a party trick. If you disable DC balancing, the performance is
// much better, and many monitors will still accept the signals as long as you
// DC couple your DVI signals.

.macro tmds_fullres_encode_loop_body ra rb
	str \ra, [r2, #ACCUM0_OFFS + INTERP1]
	str \ra, [r2, #ACCUM0_OFFS]
	ldr \ra, [r2, #PEEK2_OFFS]
	ldr \ra, [\ra]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \ra, [r2, #ACCUM1_ADD_OFFS]
#endif
	ldr \rb, [r2, #PEEK2_OFFS + INTERP1]
	ldr \rb, [\rb]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \rb, [r2, #ACCUM1_ADD_OFFS + INTERP1]
#endif
.endm

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Pixel count

.macro tmds_fullres_encode_loop_16bpp
	push {r4-r7, lr}
	mov r4, r8
	push {r4}


	lsls r2, #2
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	// DC balance defined to be 0 at start of scanline:
	movs r4, #0
	str r4, [r2, #ACCUM1_OFFS]
#if TMDS_FULLRES_NO_DC_BALANCE
	// Alternate parity between odd/even symbols if no feedback
	mvns r4, r4
#endif
	str r4, [r2, #ACCUM1_OFFS + INTERP1]

	// Keep loop start pointer in r8 so we can get a longer backward branch
	adr r4, 1f
	adds r4, #1 // god damn thumb bit why is this a thing
	mov r8, r4
	b 2f
	.align 2
1:
.rept 16
	ldmia r0!, {r4, r6}
	tmds_fullres_encode_loop_body r4 r5
	tmds_fullres_encode_loop_body r6 r7
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	beq 1f
	bx r8
1:
	pop {r4}
	mov r8, r4
	pop {r4-r7, pc}
.endm

// One copy each in X and Y, so the two cores don't step on each other
decl_func_x tmds_fullres_encode_loop_16bpp_x
	tmds_fullres_encode_loop_16bpp
decl_func_y tmds_fullres_encode_loop_16bpp_y
	tmds_fullres_encode_loop_16bpp


.macro tmds_fullres_encode_loop_body_leftshift ra rb
	// Note we apply the leftshift for INTERP0 only
	str \ra, [r2, #ACCUM0_OFFS + INTERP1]
	lsls \ra, r3
	str \ra, [r2, #ACCUM0_OFFS]
	ldr \ra, [r2, #PEEK2_OFFS]
	ldr \ra, [\ra]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \ra, [r2, #ACCUM1_ADD_OFFS]
#endif
	ldr \rb, [r2, #PEEK2_OFFS + INTERP1]
	ldr \rb, [\rb]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \rb, [r2, #ACCUM1_ADD_OFFS + INTERP1]
#endif
.endm

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Pixel count
// r3: Left shift amount

.macro tmds_fullres_encode_loop_16bpp_leftshift
	push {r4-r7, lr}
	mov r4, r8
	mov r5, r9
	push {r4-r5}

	lsls r2, #2
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	// DC balance defined to be 0 at start of scanline:
	movs r4, #0
	str r4, [r2, #ACCUM1_OFFS]
#if TMDS_FULLRES_NO_DC_BALANCE
	// Alternate parity between odd/even symbols if there's no balance feedback
	mvns r4, r4
#endif
	str r4, [r2, #ACCUM1_OFFS + INTERP1]

	adr r4, 1f
	adds r4, #1
	mov r8, r4
	b 2f
	.align 2
1:
.rept 16 // 64 pixels per iteration
	ldmia r0!, {r4, r6}
	tmds_fullres_encode_loop_body_leftshift r4 r5
	tmds_fullres_encode_loop_body_leftshift r6 r7
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	beq 1f
	bx r8
1:
	pop {r4-r5}
	mov r8, r4
	mov r9, r5
	pop {r4-r7, pc}
.endm

decl_func_x tmds_fullres_encode_loop_16bpp_leftshift_x
	tmds_fullres_encode_loop_16bpp_leftshift
decl_func_y tmds_fullres_encode_loop_16bpp_leftshift_y
	tmds_fullres_encode_loop_16bpp_leftshift


// ----------------------------------------------------------------------------
// Full-resolution 8bpp paletted encode

// Variant of tmds_fullres_encode_loop_16bpp that reads
// 8-bit wide pixels packed 4 per word.  The interpolator
// base is set to a reordered list of TMDS symbols based
// on a user colour palette.

// Two pixels input in rd[17:2]. Two symbols output in rd[19:0]. r2 contains
// interp base pointer. r7 used as temporary.
.macro tmds_palette_encode_loop_body rd
	str \rd, [r2, #ACCUM0_OFFS]
	str \rd, [r2, #ACCUM0_OFFS + INTERP1]
	ldr \rd, [r2, #PEEK2_OFFS]
	ldr \rd, [\rd]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \rd, [r2, #ACCUM1_ADD_OFFS]
#endif
	ldr r7, [r2, #PEEK2_OFFS + INTERP1]
	ldr r7, [r7]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str r7, [r2, #ACCUM1_ADD_OFFS + INTERP1]
#endif
	lsls r7, #10
	orrs \rd, r7
.endm

.macro tmds_palette_encode_loop
	push {r4-r7, lr}
	mov r4, r8
	push {r4}


	lsls r2, #1
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	// DC balance defined to be 0 at start of scanline:
	movs r4, #0
	str r4, [r2, #ACCUM1_OFFS]
#if TMDS_FULLRES_NO_DC_BALANCE
	// Alternate parity between odd/even symbols if there's no balance feedback
	mvns r4, r4
#endif
	str r4, [r2, #ACCUM1_OFFS + INTERP1]

	// Keep loop start pointer in r8 so we can get a longer backward branch
	adr r4, 1f
	adds r4, #1 // god damn thumb bit why is this a thing
	mov r8, r4
	b 2f
	.align 2
1:
.rept 10
	ldmia r0!, {r3, r5}
	lsrs r4, r3, #14
	lsls r3, #2
	lsrs r6, r5, #14
	lsls r5, #2
	tmds_palette_encode_loop_body r3
	tmds_palette_encode_loop_body r4
	tmds_palette_encode_loop_body r5
	tmds_palette_encode_loop_body r6
	stmia r1!, {r3, r4, r5, r6}
.endr
2:
	cmp r1, ip
	beq 1f
	bx r8
1:
	pop {r4}
	mov r8, r4
	pop {r4-r7, pc}
.endm

decl_func_x tmds_palette_encode_loop_x
	tmds_palette_encode_loop
decl_func_y tmds_palette_encode_loop_y
	tmds_palette_encode_loop
