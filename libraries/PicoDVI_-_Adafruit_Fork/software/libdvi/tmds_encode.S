#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
#include "dvi_config_defs.h"

// This file contains both Arm and RISC-V source, with the correct version
// selected via the __arm__ and __riscv predefined macros. The targeted Arm
// dialect is Armv6-M, and the targeted RISC-V dialect is RV32IZba

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

#if defined(__arm__) && defined(__riscv)
#error "wat"
#endif

#ifdef __arm__
.syntax unified
.cpu cortex-m0plus
.thumb
#endif

.macro decl_func_x name
.section .scratch_x.\name, "ax"
.global \name
#ifdef __arm__
.type \name,%function
.thumb_func
#endif
\name:
.endm

.macro decl_func_y name
.section .scratch_y.\name, "ax"
.global \name
#ifdef __arm__
.type \name,%function
.thumb_func
#endif
\name:
.endm

#define decl_func decl_func_x

// ----------------------------------------------------------------------------
// Pixel-doubling encoders for RGB

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Input size (pixels)
// r3: Left shift (for the *_leftshift variant only -- costs 1 cycle per 2 pixels)

#if defined(__arm__)
// Armv6-M:
.macro do_channel_16bpp r_ibase r_inout0 r_out1
	str \r_inout0, [\r_ibase, #ACCUM0_OFFS]
	ldr \r_inout0, [\r_ibase, #PEEK0_OFFS]
	ldr \r_inout0, [\r_inout0]
	ldr \r_out1, [\r_ibase, #PEEK1_OFFS]
	ldr \r_out1, [\r_out1]
.endm

.macro tmds_encode_loop_16bpp_impl leftshift
	push {r4, r5, r6, r7, lr}
	// Bounds calculation: each input pixel results in two output pixels,
	// whose two TMDS symbols are packed in a single 32-bit word. So, 4 bytes
	// out per one pixel in.
	lsls r2, #2
	add r2, r1
	mov ip, r2
	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	b 2f
.align 2
1:
.rept TMDS_ENCODE_UNROLL
	ldmia r0!, {r4, r6}
.if \leftshift
	lsls r4, r3
.endif
	do_channel_16bpp r2, r4, r5
.if \leftshift
	lsls r6, r3
.endif
	do_channel_16bpp r2, r6, r7
	stmia r1!, {r4, r5, r6, r7}
.endr
2:
	cmp r1, ip
	bne 1b
	pop {r4, r5, r6, r7, pc}
.endm

#elif defined(__riscv)
.macro do_channel_16bpp r_ibase r_inout0 r_out1
	sw \r_inout0, ACCUM0_OFFS(\r_ibase)
	// Note two halves are interleaved to avoid load->addr dependency
	lw \r_inout0, PEEK0_OFFS(\r_ibase)
	lw \r_out1, PEEK1_OFFS(\r_ibase)
	lw \r_inout0, (\r_inout0)
	lw \r_out1, (\r_out1)
.endm

.macro tmds_encode_loop_16bpp_impl leftshift
	slli a2, a2, 2
	add t0, a2, a1
	bgeu a1, t0, 2f
	li a2, SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
.align 2
1:
.set i, 0
.rept TMDS_ENCODE_UNROLL
	lw a4, 8 * i + 0(a0)
	lw a6, 8 * i + 4(a0)
.if \leftshift
	sll a4, a4, a3
	sll a6, a6, a3
.endif
	do_channel_16bpp a2, a4, a5
	do_channel_16bpp a2, a6, a7
	sw a4, 16 * i + 0(a1)
	sw a5, 16 * i + 4(a1)
	sw a6, 16 * i + 8(a1)
	sw a7, 16 * i + 12(a1)
.set i, i + 1
.endr
	addi a0, a0, 8 * TMDS_ENCODE_UNROLL
	addi a1, a1, 16 * TMDS_ENCODE_UNROLL
	bltu a1, t0, 1b
2:
	ret
.endm

#else
#error "Unknown architecture"
#endif

decl_func tmds_encode_loop_16bpp
tmds_encode_loop_16bpp_impl 0

decl_func tmds_encode_loop_16bpp_leftshift
tmds_encode_loop_16bpp_impl 1

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Input size (pixels)
// r3: Left shift amount (for the *_leftshift variant of the function)
//
// Note that only the data written to interp0 (pixel 0, 1) is leftshifted, not
// the data written to interp1 (pixel 2, 3). Otherwise we always lose MSBs, as
// the LUT offset MSB is at bit 8, so pixel 0 always requires some left shift,
// since its channel MSBs are no greater than 7.

#if defined(__arm__)
.macro tmds_encode_loop_8bpp_impl leftshift
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
.if \leftshift
	lsls r4, r3
.endif
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
.endm

#elif defined(__riscv)
.macro tmds_encode_loop_8bpp_impl leftshift
	slli a2, a2, 2
	add a2, a2, a1
	bgeu a1, a2, 2f
	mv t0, a2
	li a2, SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
.align 2
1:
.set i, 0
.rept TMDS_ENCODE_UNROLL
	lw a4, 4 * i(a0)
	sw a4, ACCUM0_OFFS + INTERP1(a2)
.if \leftshift
	sll a4, a4, a3
.endif
	sw a4, ACCUM0_OFFS(a2)
	lw a4, PEEK0_OFFS(a2)
	lw a5, PEEK1_OFFS(a2)
	lw a4, (a4)
	lw a5, (a5)
	lw a6, PEEK0_OFFS + INTERP1(a2)
	lw a7, PEEK1_OFFS + INTERP1(a2)
	lw a6, (a6)
	lw a7, (a7)
	sw a4, 16 * i +  0(a1)
	sw a5, 16 * i +  4(a1)
	sw a6, 16 * i +  8(a1)
	sw a7, 16 * i + 12(a1)
.set i, i + 1
.endr
	addi a0, a0, TMDS_ENCODE_UNROLL * 4
	addi a1, a1, TMDS_ENCODE_UNROLL * 16
	bltu a1, t0, 1b
2:
	ret
.endm

#else
#error "Unknown architecture"
#endif

decl_func tmds_encode_loop_8bpp
tmds_encode_loop_8bpp_impl 0
decl_func tmds_encode_loop_8bpp_leftshift
tmds_encode_loop_8bpp_impl 1

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

#if defined(__arm__)
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

#elif defined(__riscv)
// TODO the register allocation is not optimal here for code size
.macro tmds_encode_1bpp_body shift_instr0 shamt0 shift_instr1 shamt1
	\shift_instr0 a4, a2, \shamt0
	and a4, a4, a3
	add a4, a4, t1
	lw a5, 4(a4)
	lw a4, 0(a4)
	\shift_instr1 a6, a2, \shamt1
	and a6, a6, a3
	add a6, a6, t1
	lw a7, 4(a6)
	lw a6, 0(a6)
	sw a4, 0(a1)
	sw a5, 4(a1)
	sw a6, 8(a1)
	sw a7, 12(a1)
	addi a1, a1, 16
.endm

// a0: input buffer (word-aligned)
// a1: output buffer (word-aligned)
// a2: output pixel count
decl_func tmds_encode_1bpp
	slli a2, a2, 1
	add t0, a2, a1
	la t1, tmds_1bpp_table
	// Mask: 4 bit index, 8 bytes per entry
	li a3, 0x78
	bgeu a1, t0, 2f
1:
	lw a2, (a0)
	addi a0, a0, 4
#if !DVI_1BPP_BIT_REVERSE
	tmds_encode_1bpp_body slli 3  srli 1
	tmds_encode_1bpp_body srli 5  srli 9
	tmds_encode_1bpp_body srli 13 srli 17
	tmds_encode_1bpp_body srli 21 srli 25
#else
	tmds_encode_1bpp_body srli 1   slli 3
	tmds_encode_1bpp_body srli 9   srli 5
	tmds_encode_1bpp_body srli 17  srli 13
	tmds_encode_1bpp_body srli 25  srli 21
#endif
	bltu a1, t0, 1b
2:
	ret

#else
#error "Unknown architecture"
#endif

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

#if defined(__arm__)
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

#elif defined(__riscv)
// Table base pointer in a0. Input pixels in a2.
.macro encode_2bpp_body shift_instr shamt rd
	\shift_instr \rd, a2, \shamt
	and \rd, \rd, a3
	add \rd, \rd, a0
	lw \rd, (\rd)
.endm

// a0: input buffer (word-aligned)
// a1: output buffer (word-aligned)
// a2: output pixel count
decl_func tmds_encode_2bpp
	mv t1, a0
	la a0, tmds_2bpp_table
	// Mask: 4-bit index into 4-byte entries.
	li a3, 0x3c
	// Limit pointer: 1 word per 2 pixels
	slli a2, a2, 1
	add t0, a2, a1
	bgeu a1, t0, 1b
1:
	lw a2, (t1)
	addi t1, t1, 4
	encode_2bpp_body slli 2  a4
	encode_2bpp_body srli 2  a5
	encode_2bpp_body srli 6  a6
	encode_2bpp_body srli 10 a7
	sw a4, 0(a1)
	sw a5, 4(a1)
	sw a6, 8(a1)
	sw a7, 12(a1)
	encode_2bpp_body srli 14 a4
	encode_2bpp_body srli 18 a5
	encode_2bpp_body srli 22 a6
	encode_2bpp_body srli 26 a7
	sw a4, 16(a1)
	sw a5, 20(a1)
	sw a6, 24(a1)
	sw a7, 28(a1)
	addi a1, a1, 32
	bltu a1, t0, 1b
2:
	ret

#else
#error "Unknown architecture"
#endif

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

#if defined(__arm__)
.macro tmds_fullres_encode_loop_body leftshift ra rb
	str \ra, [r2, #ACCUM0_OFFS + INTERP1]
.if \leftshift
	lsls \ra, r3
.endif
	str \ra, [r2, #ACCUM0_OFFS]
	// Loads interleaved to avoid rdata->addr stall on M33
	ldr \ra, [r2, #PEEK2_OFFS]
	ldr \rb, [r2, #PEEK2_OFFS + INTERP1]
	ldr \ra, [\ra]
	ldr \rb, [\rb]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \ra, [r2, #ACCUM1_ADD_OFFS]
	str \rb, [r2, #ACCUM1_ADD_OFFS + INTERP1]
#endif
.endm

// r0: Input buffer (word-aligned)
// r1: Output buffer (word-aligned)
// r2: Pixel count
// r3: Left shift amount

.macro tmds_fullres_encode_loop_16bpp leftshift
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
	tmds_fullres_encode_loop_body \leftshift r4 r5
	tmds_fullres_encode_loop_body \leftshift r6 r7
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

#elif defined(__riscv)

.macro tmds_fullres_encode_loop_body leftshift ra rb
	sw \ra, ACCUM0_OFFS + INTERP1(a2)
.if \leftshift
	sll \ra, \ra, a3
.endif
	sw \ra, ACCUM0_OFFS(a2)
	lw \ra, PEEK2_OFFS(a2)
	lw \rb, PEEK2_OFFS + INTERP1(a2)
	lw \ra, (\ra)
	lw \rb, (\rb)
#if !TMDS_FULLRES_NO_DC_BALANCE
	sw \ra, ACCUM1_ADD_OFFS(a2)
	sw \rb, ACCUM1_ADD_OFFS + INTERP1(a2)
#endif
.endm

// a0: Input buffer (word-aligned)
// a1: Output buffer (word-aligned)
// a2: Pixel count
// a3: Left shift amount

.macro tmds_fullres_encode_loop_16bpp leftshift
	sh2add t0, a2, a1
	li a2, SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
	// DC balance defined to be 0 at start of scanline:
	li a4, 0
	sw a4, ACCUM1_OFFS(a2)
#if TMDS_FULLRES_NO_DC_BALANCE
	// Alternate parity between odd/even symbols if no feedback
	li a4, -1
#endif
	sw a4, ACCUM1_OFFS + INTERP1(a2)

	bgeu a1, t0, 2f
	.align 2
1:
.set i, 0
.rept 16
	lw a4, 8 * i + 0(a0)
	lw a6, 8 * i + 4(a0)
	tmds_fullres_encode_loop_body \leftshift a4 a5
	tmds_fullres_encode_loop_body \leftshift a6 a7
	sw a4, 16 * i +  0(a1)
	sw a5, 16 * i +  4(a1)
	sw a6, 16 * i +  8(a1)
	sw a7, 16 * i + 12(a1)
.set i, i + 1
.endr
	addi a0, a0,  8 * i
	addi a1, a1, 16 * i
	bltu a1, t0, 1b
2:
	ret
.endm

#else
#error "Unknown architecture"
#endif

// One copy each in X and Y, so the two cores don't step on each other
decl_func_x tmds_fullres_encode_loop_16bpp_x
	tmds_fullres_encode_loop_16bpp 0
decl_func_y tmds_fullres_encode_loop_16bpp_y
	tmds_fullres_encode_loop_16bpp 0

decl_func_x tmds_fullres_encode_loop_16bpp_leftshift_x
	tmds_fullres_encode_loop_16bpp 1
decl_func_y tmds_fullres_encode_loop_16bpp_leftshift_y
	tmds_fullres_encode_loop_16bpp 1

// ----------------------------------------------------------------------------
// Full-resolution 8bpp paletted encode

// Variant of tmds_fullres_encode_loop_16bpp that reads
// 8-bit wide pixels packed 4 per word.  The interpolator
// base is set to a reordered list of TMDS symbols based
// on a user colour palette.

#ifdef __arm__
// Two pixels input in rd[17:2]. Two symbols output in rd[19:0]. r2 contains
// interp base pointer. r7 used as temporary.
.macro tmds_palette_encode_loop_body rd
	str \rd, [r2, #ACCUM0_OFFS]
	str \rd, [r2, #ACCUM0_OFFS + INTERP1]
	// Loads interleaved to avoid rdata->addr stall on M33
	ldr \rd, [r2, #PEEK2_OFFS]
	ldr r7, [r2, #PEEK2_OFFS + INTERP1]
	ldr \rd, [\rd]
	ldr r7, [r7]
#if !TMDS_FULLRES_NO_DC_BALANCE
	str \rd, [r2, #ACCUM1_ADD_OFFS]
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

#elif defined(__riscv)

// Two pixels input in rd[17:2]. Two symbols output in rd[19:0]. a2 contains
// interp base pointer. a5 used as temporary.
.macro tmds_palette_encode_loop_body rd
	sw \rd, ACCUM0_OFFS(a2)
	sw \rd, ACCUM0_OFFS + INTERP1(a2)
	lw \rd, PEEK2_OFFS(a2)
	lw a5, PEEK2_OFFS + INTERP1(a2)
	lw \rd, (\rd)
	lw a5, (a5)
#if !TMDS_FULLRES_NO_DC_BALANCE
	sw \rd, ACCUM1_ADD_OFFS(a2)
	sw a5, ACCUM1_ADD_OFFS + INTERP1(a2)
#endif
	slli a5, a5, 10
	or \rd, \rd, a5
.endm

.macro tmds_palette_encode_loop
	mv t1, s0
	sh1add t0, a2, a1
	li a2, SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
	// DC balance defined to be 0 at start of scanline:
	li a4, 0
	sw a4, ACCUM1_OFFS(a2)
#if TMDS_FULLRES_NO_DC_BALANCE
	// Alternate parity between odd/even symbols if there's no balance feedback
	li a4, -1
#endif
	sw a4, ACCUM1_OFFS + INTERP1(a2)

	bgeu a1, t0, 2f
	.align 2
1:
.set i, 0
.rept 10
	lw a3, 8 * i + 0(a0)
	lw s0, 8 * i + 4(a0)
	srli a4, a3, 14
	slli a3, a3, 2
	tmds_palette_encode_loop_body a3
	tmds_palette_encode_loop_body a4
	sw a3, 16 * i + 0(a1)
	sw a4, 16 * i + 4(a1)
	srli a4, s0, 14
	slli s0, s0, 2
	tmds_palette_encode_loop_body s0
	tmds_palette_encode_loop_body a4
	sw s0, 16 * i + 8(a1)
	sw a4, 16 * i + 12(a1)
.set i, i + 1
.endr
	addi a0, a0, 8 * i
	addi a1, a1, 16 * i
	bltu a1, t0, 1b
2:
	mv s0, t1
	ret
.endm


#endif

decl_func_x tmds_palette_encode_loop_x
	tmds_palette_encode_loop
decl_func_y tmds_palette_encode_loop_y
	tmds_palette_encode_loop

// ----------------------------------------------------------------------------
// Hand-cranking loops for SIO TMDS encoders

#if DVI_USE_SIO_TMDS_ENCODER

#if defined(__arm__)

// r0: input buffer (word-aligned)
// r1: output buffer (word-aligned)
// r2: pixel count

.macro tmds_encode_sio_loop size_ratio peek

// For larger load/store offsets at high ratios/unroll:
.cpu cortex-m33

.if \size_ratio > 4 * TMDS_ENCODE_UNROLL
.set unroll, 1
.else
.set unroll, 4 * TMDS_ENCODE_UNROLL / \size_ratio
.endif

.if \peek
.set even_offset_adj, (SIO_TMDS_PEEK_SINGLE_OFFSET - SIO_TMDS_POP_SINGLE_OFFSET)
.else
.set even_offset_adj, 0
.endif

	push {r4, lr}
#if DVI_SYMBOLS_PER_WORD == 1
	lsls r2, r2, #2
#else
	lsls r2, r2, #1
#endif
	adds r2, r1
	ldr r3, =SIO_BASE + SIO_TMDS_CTRL_OFFSET
	b 2f
1:
.set i, 0
.rept unroll
	ldr r4, [r0, #i * 4]
	str r4, [r3, #SIO_TMDS_WDATA_OFFSET - SIO_TMDS_CTRL_OFFSET]
.set j, 0
.rept \size_ratio
.set offset_adj, even_offset_adj * ((1 + j + \size_ratio * i) & 0x1)
#if DVI_SYMBOLS_PER_WORD == 2
	ldr r4, [r3, #offset_adj + SIO_TMDS_POP_DOUBLE_L0_OFFSET - SIO_TMDS_CTRL_OFFSET]
#else
	ldr r4, [r3, #offset_adj + SIO_TMDS_POP_SINGLE_OFFSET - SIO_TMDS_CTRL_OFFSET]
#endif
	str r4, [r1, #4 * (j + i * \size_ratio)]
.set j, j + 1
.endr
.set i, i + 1
.endr
	adds r0, 4 * unroll
	adds r1, 4 * unroll * \size_ratio
2:
	cmp r1, r2
	blo 1b
	pop {r4, pc}

.cpu cortex-m0plus
.endm

#elif defined(__riscv)

// a0: input buffer (word-aligned)
// a1: output buffer (word-aligned)
// a2: pixel count

.macro tmds_encode_sio_loop size_ratio peek

.if \size_ratio > 4 * TMDS_ENCODE_UNROLL
.set unroll, 1
.else
.set unroll, 4 * TMDS_ENCODE_UNROLL / \size_ratio
.endif

.if \peek
.set even_offset_adj, (SIO_TMDS_PEEK_SINGLE_OFFSET - SIO_TMDS_POP_SINGLE_OFFSET)
.else
.set even_offset_adj, 0
.endif

#if DVI_SYMBOLS_PER_WORD == 1
	sh2add a2, a2, a1
#else
	sh1add a2, a2, a1
#endif
	li a3, SIO_BASE + SIO_TMDS_CTRL_OFFSET
	bgeu a1, a2, 2f
1:
.set i, 0
.rept unroll
	lw a4, i * 4(a0)
	sw a4, SIO_TMDS_WDATA_OFFSET - SIO_TMDS_CTRL_OFFSET(a3)
.set j, 0
.rept \size_ratio
.set offset_adj, even_offset_adj * ((1 + j + \size_ratio * i) & 0x1)
#if DVI_SYMBOLS_PER_WORD == 2
	lw a4, offset_adj + SIO_TMDS_POP_DOUBLE_L0_OFFSET - SIO_TMDS_CTRL_OFFSET(a3)
#else
	lw a4, offset_adj + SIO_TMDS_POP_SINGLE_OFFSET - SIO_TMDS_CTRL_OFFSET(a3)
#endif
	sw a4, 4 * (j + i * \size_ratio)(a1)
.set j, j + 1
.endr
.set i, i + 1
.endr
	addi a0, a0, 4 * unroll
	addi a1, a1, 4 * unroll * \size_ratio
	bltu a1, a2, 1b
2:
	ret
.endm

#else
#error "Unknown architecture"
#endif

// For DVI_SYMBOLS_PER_WORD == 2, the ratio of output : input buffer size is:
//
// Bits/pixel | Ratio (with hdouble) | Ratio (no hdouble)
// -----------+----------------------+-------------------
//          1 |                   32 |                 16
//          2 |                   16 |                  8
//          4 |                    8 |                  4
//          8 |                    4 |                  2
//         16 |                    2 |                  1
//
// For DVI_SYMBOLS_PER_WORD == 1, these ratios are doubled.

// poppop variants will read from a xxx_POP register for every output word
decl_func tmds_encode_sio_loop_poppop_ratio1
	tmds_encode_sio_loop 1, 0
decl_func tmds_encode_sio_loop_poppop_ratio2
	tmds_encode_sio_loop 2, 0
decl_func tmds_encode_sio_loop_poppop_ratio4
	tmds_encode_sio_loop 4, 0
decl_func tmds_encode_sio_loop_poppop_ratio8
	tmds_encode_sio_loop 8, 0
decl_func tmds_encode_sio_loop_poppop_ratio16
	tmds_encode_sio_loop 16, 0
decl_func tmds_encode_sio_loop_poppop_ratio32
	tmds_encode_sio_loop 32, 0
decl_func tmds_encode_sio_loop_poppop_ratio64
	tmds_encode_sio_loop 64, 0

// peekpop variants will read alternately from xxx_PEEK and xxx_POP: this is
// needed for pixel-doubled output when DVI_PIXELS_PER_WORD == 1 (note the
// POP value is different from the PEEK value, as it's the same data but with
// different running DC balance)
decl_func tmds_encode_sio_loop_peekpop_ratio1
	tmds_encode_sio_loop 1, 1
decl_func tmds_encode_sio_loop_peekpop_ratio2
	tmds_encode_sio_loop 2, 1
decl_func tmds_encode_sio_loop_peekpop_ratio4
	tmds_encode_sio_loop 4, 1
decl_func tmds_encode_sio_loop_peekpop_ratio8
	tmds_encode_sio_loop 8, 1
decl_func tmds_encode_sio_loop_peekpop_ratio16
	tmds_encode_sio_loop 16, 1
decl_func tmds_encode_sio_loop_peekpop_ratio32
	tmds_encode_sio_loop 32, 1
decl_func tmds_encode_sio_loop_peekpop_ratio64
	tmds_encode_sio_loop 64, 1

#endif
