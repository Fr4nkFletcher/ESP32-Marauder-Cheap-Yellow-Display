#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

#ifndef __riscv
.syntax unified
.cpu cortex-m0plus
.thumb
#endif

// Using the following:
//
// - A font stored as a 1bpp bitmap, with row 0 of each character stored in
//   one contiguous array, then row 1, etc, where each character is 8 bits
//   wide
//
// - A character buffer
//
// - A colour buffer for each of R, G, B (so 3 planes total), each buffer
//   storing a 2-bit foreground and background colour for each character
//
// Generate encoded TMDS buffers, fast enough to fit all the encode on one
// core, and with small memory footprint (so no framebuffer of any depth!) The
// main trick here is a LUT with an 8 bit index, composed of 4x1bpp pixels
// (the 4 LSBs of the index) and a 2x2-bit palette (the four MSBs of the
// index). Each LUT entry is 4 TMDS symbols, so 2 32-bit words, giving a total
// table size of 2 kB.

// Offsets suitable for ldr/str (must be <= 0x7c):
#define ACCUM0_OFFS     (SIO_INTERP0_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_OFFS     (SIO_INTERP0_ACCUM1_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_ADD_OFFS (SIO_INTERP0_ACCUM1_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK0_OFFS      (SIO_INTERP0_PEEK_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK1_OFFS      (SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK2_OFFS      (SIO_INTERP0_PEEK_FULL_OFFSET  - SIO_INTERP0_ACCUM0_OFFSET)
#define INTERP1         (SIO_INTERP1_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)

// There is no vertical repeat, so the the budget (ignoring DMA IRQs) is 8000
// cycles per 640 pixels, and there are three symbols to be generated per
// pixel, so 4.17 cyc/pix.


// Once in the loop:
// r0 contains character buffer pointer (only updated once per 8 chars)
// r1 contains 8 2-colour 2bpp palettes, enough for 8 characters
// r2 contains output buffer pointer
// r3 contains a mask for the colour lookup bits
// r4-r7 are for scratch + pixels
// r8 contains a pointer to the font bitmap for this scanline.
// r9 contains the TMDS LUT base.
.macro do_char charbuf_offs colour_shift_instr colour_shamt
#ifndef __riscv
	// Get 8x font bits for next character, put 4 LSBs in bits 6:3 of r4 (so
	// scaled to 8-byte LUT entries), and 4 MSBs in bits 6:3 of r6.
	ldrb r4, [r0, #\charbuf_offs]                                     // 2 (note these cycle
	add r4, r8                                                        // 1  counts are for M0+
	ldrb r4, [r4]                                                     // 2  and are a little
	lsrs r6, r4, #4                                                   // 1  pessimistic on M33)
	lsls r6, #3                                                       // 1
	lsls r4, #28                                                      // 1
	lsrs r4, #25                                                      // 1

	// Get colour bits, add to TMDS LUT base and font bits
	\colour_shift_instr r5, r1, #\colour_shamt                        // 1
	ands r5, r3                                                       // 1
	add r5, r9                                                        // 1
	add r4, r5                                                        // 1
	add r6, r5                                                        // 1

	// Look up and write out 8 TMDS symbols
	ldmia r4, {r4, r5}                                                // 3
	ldmia r6, {r6, r7}                                                // 3
	stmia r2!, {r4-r7}                                                // 5
#else
	lbu a4, \charbuf_offs(a0)                                         // 1
	\colour_shift_instr a5, a1, \colour_shamt                         // 1
	add a4, a4, t1                                                    // 1
	lbu a4, (a4)                                                      // 2
	srli a6, a4, 4                                                    // 1
	andi a4, a4, 0xf                                                  // 1

	// Get colour bits, add to TMDS LUT base and font bits
	and a5, a5, a3                                                    // 1
	add a5, a5, t2                                                    // 1
	sh3add a4, a4, a5                                                 // 1
	sh3add a6, a6, a5                                                 // 1

	// Look up and write out 8 TMDS symbols
	lw a5, 4(a4)                                                      // 1
	lw a4, 0(a4)                                                      // 1
	lw a7, 4(a6)                                                      // 1
	lw a6, 0(a6)                                                      // 1
	sw a4, 0(a2)                                                      // 1
	sw a5, 4(a2)                                                      // 1
	sw a6, 8(a2)                                                      // 1
	sw a7, 12(a2)                                                     // 1
	addi a2, a2, 16                                                   // 1
#endif
.endm


// r0 is character buffer
// r1 is colour buffer
// r2 is output TMDS buffer
// r3 is pixel count
// First stack argument is the font bitmap for this scanline.

.section .scratch_x.tmds_encode_font_2bpp, "ax"
.global tmds_encode_font_2bpp
#ifndef __riscv
.type tmds_encode_font_2bpp,%function
.thumb_func
#endif
tmds_encode_font_2bpp:
#ifndef __riscv
	push {r4-r7, lr}
	mov r4, r8
	mov r5, r9
	mov r6, r10
	push {r4-r6}

	lsls r3, #1
	add r3, r2
	mov ip, r3
	ldr r3, =(0xf0 * 8)

	ldr r7, [sp, #32] // 8 words saved, so 32-byte offset to first stack argument
	mov r8, r7
	ldr r7, =palettised_1bpp_tables
	mov r9, r7

	mov r10, r1

	b 2f
1:
	mov r4, r10
	ldmia r4!, {r1}
	mov r10, r4
	do_char 0 lsls 7
	do_char 1 lsls 3
	do_char 2 lsrs 1
	do_char 3 lsrs 5
	do_char 4 lsrs 9
	do_char 5 lsrs 13
	do_char 6 lsrs 17
	do_char 7 lsrs 21
	adds r0, #8
2:
	cmp r2, ip
	blo 1b

	pop {r4-r6}
	mov r8, r4
	mov r9, r5
	mov r10, r6
	pop {r4-r7, pc}

#else

	sh1add t0, a3, a2
	li a3, 0xf0 * 8

	mv t1, a4
	la t2, palettised_1bpp_tables
	mv t3, a1

	bgeu a2, t0, 2f
1:
	lw a1, (t3)
	addi t3, t3, 4
	do_char 0 slli 7
	do_char 1 slli 3
	do_char 2 srli 1
	do_char 3 srli 5
	do_char 4 srli 9
	do_char 5 srli 13
	do_char 6 srli 17
	do_char 7 srli 21
	addi a0, a0, 8
	bltu a2, t0, 1b
2:
	ret
#endif

// Table generation:
//	levels_2bpp_even = [0x05, 0x50, 0xaf, 0xfa]
//	levels_2bpp_odd  = [0x04, 0x51, 0xae, 0xfb]
//	
//	def level(bg, fg, x, pix):
//		index = fg if pix & 1 << x else bg
//		return (levels_2bpp_odd if x & 1 else levels_2bpp_even)[index]
//	
//	for background in range(4):
//		for foreground in range(4):
//			print(f"// background, foreground = {background:02b}, {foreground:02b}")
//			for pixrun in range(16):
//				sym = list(enc.encode(level(background, foreground, x, pixrun), 0, 1) for x in range(4))
//				assert(enc.imbalance == 0)
//				print(f".word 0x{sym[1] << 10 | sym[0]:05x}, 0x{sym[3] << 10 | sym[2]:05x} // {pixrun:04b}")

.section .scratch_x.palettised_1bpp_tables, "a"
.align 2
palettised_1bpp_tables:
	// background, foreground = 00, 00
	.word 0x7f103, 0x7f103 // 0000
	.word 0x7f103, 0x7f103 // 0001
	.word 0x7f103, 0x7f103 // 0010
	.word 0x7f103, 0x7f103 // 0011
	.word 0x7f103, 0x7f103 // 0100
	.word 0x7f103, 0x7f103 // 0101
	.word 0x7f103, 0x7f103 // 0110
	.word 0x7f103, 0x7f103 // 0111
	.word 0x7f103, 0x7f103 // 1000
	.word 0x7f103, 0x7f103 // 1001
	.word 0x7f103, 0x7f103 // 1010
	.word 0x7f103, 0x7f103 // 1011
	.word 0x7f103, 0x7f103 // 1100
	.word 0x7f103, 0x7f103 // 1101
	.word 0x7f103, 0x7f103 // 1110
	.word 0x7f103, 0x7f103 // 1111
	// background, foreground = 00, 01
	.word 0x7f103, 0x7f103 // 0000
	.word 0x7f130, 0x7f103 // 0001
	.word 0x73d03, 0x7f103 // 0010
	.word 0x73d30, 0x7f103 // 0011
	.word 0x7f103, 0x7f130 // 0100
	.word 0x7f130, 0x7f130 // 0101
	.word 0x73d03, 0x7f130 // 0110
	.word 0x73d30, 0x7f130 // 0111
	.word 0x7f103, 0x73d03 // 1000
	.word 0x7f130, 0x73d03 // 1001
	.word 0x73d03, 0x73d03 // 1010
	.word 0x73d30, 0x73d03 // 1011
	.word 0x7f103, 0x73d30 // 1100
	.word 0x7f130, 0x73d30 // 1101
	.word 0x73d03, 0x73d30 // 1110
	.word 0x73d30, 0x73d30 // 1111
	// background, foreground = 00, 10
	.word 0x7f103, 0x7f103 // 0000
	.word 0x7f230, 0x7f103 // 0001
	.word 0xb3d03, 0x7f103 // 0010
	.word 0xb3e30, 0x7f103 // 0011
	.word 0x7f103, 0x7f230 // 0100
	.word 0x7f230, 0x7f230 // 0101
	.word 0xb3d03, 0x7f230 // 0110
	.word 0xb3e30, 0x7f230 // 0111
	.word 0x7f103, 0xb3d03 // 1000
	.word 0x7f230, 0xb3d03 // 1001
	.word 0xb3d03, 0xb3d03 // 1010
	.word 0xb3e30, 0xb3d03 // 1011
	.word 0x7f103, 0xb3e30 // 1100
	.word 0x7f230, 0xb3e30 // 1101
	.word 0xb3d03, 0xb3e30 // 1110
	.word 0xb3e30, 0xb3e30 // 1111
	// background, foreground = 00, 11
	.word 0x7f103, 0x7f103 // 0000
	.word 0x7f203, 0x7f103 // 0001
	.word 0xbf103, 0x7f103 // 0010
	.word 0xbf203, 0x7f103 // 0011
	.word 0x7f103, 0x7f203 // 0100
	.word 0x7f203, 0x7f203 // 0101
	.word 0xbf103, 0x7f203 // 0110
	.word 0xbf203, 0x7f203 // 0111
	.word 0x7f103, 0xbf103 // 1000
	.word 0x7f203, 0xbf103 // 1001
	.word 0xbf103, 0xbf103 // 1010
	.word 0xbf203, 0xbf103 // 1011
	.word 0x7f103, 0xbf203 // 1100
	.word 0x7f203, 0xbf203 // 1101
	.word 0xbf103, 0xbf203 // 1110
	.word 0xbf203, 0xbf203 // 1111
	// background, foreground = 01, 00
	.word 0x73d30, 0x73d30 // 0000
	.word 0x73d03, 0x73d30 // 0001
	.word 0x7f130, 0x73d30 // 0010
	.word 0x7f103, 0x73d30 // 0011
	.word 0x73d30, 0x73d03 // 0100
	.word 0x73d03, 0x73d03 // 0101
	.word 0x7f130, 0x73d03 // 0110
	.word 0x7f103, 0x73d03 // 0111
	.word 0x73d30, 0x7f130 // 1000
	.word 0x73d03, 0x7f130 // 1001
	.word 0x7f130, 0x7f130 // 1010
	.word 0x7f103, 0x7f130 // 1011
	.word 0x73d30, 0x7f103 // 1100
	.word 0x73d03, 0x7f103 // 1101
	.word 0x7f130, 0x7f103 // 1110
	.word 0x7f103, 0x7f103 // 1111
	// background, foreground = 01, 01
	.word 0x73d30, 0x73d30 // 0000
	.word 0x73d30, 0x73d30 // 0001
	.word 0x73d30, 0x73d30 // 0010
	.word 0x73d30, 0x73d30 // 0011
	.word 0x73d30, 0x73d30 // 0100
	.word 0x73d30, 0x73d30 // 0101
	.word 0x73d30, 0x73d30 // 0110
	.word 0x73d30, 0x73d30 // 0111
	.word 0x73d30, 0x73d30 // 1000
	.word 0x73d30, 0x73d30 // 1001
	.word 0x73d30, 0x73d30 // 1010
	.word 0x73d30, 0x73d30 // 1011
	.word 0x73d30, 0x73d30 // 1100
	.word 0x73d30, 0x73d30 // 1101
	.word 0x73d30, 0x73d30 // 1110
	.word 0x73d30, 0x73d30 // 1111
	// background, foreground = 01, 10
	.word 0x73d30, 0x73d30 // 0000
	.word 0x73e30, 0x73d30 // 0001
	.word 0xb3d30, 0x73d30 // 0010
	.word 0xb3e30, 0x73d30 // 0011
	.word 0x73d30, 0x73e30 // 0100
	.word 0x73e30, 0x73e30 // 0101
	.word 0xb3d30, 0x73e30 // 0110
	.word 0xb3e30, 0x73e30 // 0111
	.word 0x73d30, 0xb3d30 // 1000
	.word 0x73e30, 0xb3d30 // 1001
	.word 0xb3d30, 0xb3d30 // 1010
	.word 0xb3e30, 0xb3d30 // 1011
	.word 0x73d30, 0xb3e30 // 1100
	.word 0x73e30, 0xb3e30 // 1101
	.word 0xb3d30, 0xb3e30 // 1110
	.word 0xb3e30, 0xb3e30 // 1111
	// background, foreground = 01, 11
	.word 0x73d30, 0x73d30 // 0000
	.word 0x73e03, 0x73d30 // 0001
	.word 0xbf130, 0x73d30 // 0010
	.word 0xbf203, 0x73d30 // 0011
	.word 0x73d30, 0x73e03 // 0100
	.word 0x73e03, 0x73e03 // 0101
	.word 0xbf130, 0x73e03 // 0110
	.word 0xbf203, 0x73e03 // 0111
	.word 0x73d30, 0xbf130 // 1000
	.word 0x73e03, 0xbf130 // 1001
	.word 0xbf130, 0xbf130 // 1010
	.word 0xbf203, 0xbf130 // 1011
	.word 0x73d30, 0xbf203 // 1100
	.word 0x73e03, 0xbf203 // 1101
	.word 0xbf130, 0xbf203 // 1110
	.word 0xbf203, 0xbf203 // 1111
	// background, foreground = 10, 00
	.word 0xb3e30, 0xb3e30 // 0000
	.word 0xb3d03, 0xb3e30 // 0001
	.word 0x7f230, 0xb3e30 // 0010
	.word 0x7f103, 0xb3e30 // 0011
	.word 0xb3e30, 0xb3d03 // 0100
	.word 0xb3d03, 0xb3d03 // 0101
	.word 0x7f230, 0xb3d03 // 0110
	.word 0x7f103, 0xb3d03 // 0111
	.word 0xb3e30, 0x7f230 // 1000
	.word 0xb3d03, 0x7f230 // 1001
	.word 0x7f230, 0x7f230 // 1010
	.word 0x7f103, 0x7f230 // 1011
	.word 0xb3e30, 0x7f103 // 1100
	.word 0xb3d03, 0x7f103 // 1101
	.word 0x7f230, 0x7f103 // 1110
	.word 0x7f103, 0x7f103 // 1111
	// background, foreground = 10, 01
	.word 0xb3e30, 0xb3e30 // 0000
	.word 0xb3d30, 0xb3e30 // 0001
	.word 0x73e30, 0xb3e30 // 0010
	.word 0x73d30, 0xb3e30 // 0011
	.word 0xb3e30, 0xb3d30 // 0100
	.word 0xb3d30, 0xb3d30 // 0101
	.word 0x73e30, 0xb3d30 // 0110
	.word 0x73d30, 0xb3d30 // 0111
	.word 0xb3e30, 0x73e30 // 1000
	.word 0xb3d30, 0x73e30 // 1001
	.word 0x73e30, 0x73e30 // 1010
	.word 0x73d30, 0x73e30 // 1011
	.word 0xb3e30, 0x73d30 // 1100
	.word 0xb3d30, 0x73d30 // 1101
	.word 0x73e30, 0x73d30 // 1110
	.word 0x73d30, 0x73d30 // 1111
	// background, foreground = 10, 10
	.word 0xb3e30, 0xb3e30 // 0000
	.word 0xb3e30, 0xb3e30 // 0001
	.word 0xb3e30, 0xb3e30 // 0010
	.word 0xb3e30, 0xb3e30 // 0011
	.word 0xb3e30, 0xb3e30 // 0100
	.word 0xb3e30, 0xb3e30 // 0101
	.word 0xb3e30, 0xb3e30 // 0110
	.word 0xb3e30, 0xb3e30 // 0111
	.word 0xb3e30, 0xb3e30 // 1000
	.word 0xb3e30, 0xb3e30 // 1001
	.word 0xb3e30, 0xb3e30 // 1010
	.word 0xb3e30, 0xb3e30 // 1011
	.word 0xb3e30, 0xb3e30 // 1100
	.word 0xb3e30, 0xb3e30 // 1101
	.word 0xb3e30, 0xb3e30 // 1110
	.word 0xb3e30, 0xb3e30 // 1111
	// background, foreground = 10, 11
	.word 0xb3e30, 0xb3e30 // 0000
	.word 0xb3e03, 0xb3e30 // 0001
	.word 0xbf230, 0xb3e30 // 0010
	.word 0xbf203, 0xb3e30 // 0011
	.word 0xb3e30, 0xb3e03 // 0100
	.word 0xb3e03, 0xb3e03 // 0101
	.word 0xbf230, 0xb3e03 // 0110
	.word 0xbf203, 0xb3e03 // 0111
	.word 0xb3e30, 0xbf230 // 1000
	.word 0xb3e03, 0xbf230 // 1001
	.word 0xbf230, 0xbf230 // 1010
	.word 0xbf203, 0xbf230 // 1011
	.word 0xb3e30, 0xbf203 // 1100
	.word 0xb3e03, 0xbf203 // 1101
	.word 0xbf230, 0xbf203 // 1110
	.word 0xbf203, 0xbf203 // 1111
	// background, foreground = 11, 00
	.word 0xbf203, 0xbf203 // 0000
	.word 0xbf103, 0xbf203 // 0001
	.word 0x7f203, 0xbf203 // 0010
	.word 0x7f103, 0xbf203 // 0011
	.word 0xbf203, 0xbf103 // 0100
	.word 0xbf103, 0xbf103 // 0101
	.word 0x7f203, 0xbf103 // 0110
	.word 0x7f103, 0xbf103 // 0111
	.word 0xbf203, 0x7f203 // 1000
	.word 0xbf103, 0x7f203 // 1001
	.word 0x7f203, 0x7f203 // 1010
	.word 0x7f103, 0x7f203 // 1011
	.word 0xbf203, 0x7f103 // 1100
	.word 0xbf103, 0x7f103 // 1101
	.word 0x7f203, 0x7f103 // 1110
	.word 0x7f103, 0x7f103 // 1111
	// background, foreground = 11, 01
	.word 0xbf203, 0xbf203 // 0000
	.word 0xbf130, 0xbf203 // 0001
	.word 0x73e03, 0xbf203 // 0010
	.word 0x73d30, 0xbf203 // 0011
	.word 0xbf203, 0xbf130 // 0100
	.word 0xbf130, 0xbf130 // 0101
	.word 0x73e03, 0xbf130 // 0110
	.word 0x73d30, 0xbf130 // 0111
	.word 0xbf203, 0x73e03 // 1000
	.word 0xbf130, 0x73e03 // 1001
	.word 0x73e03, 0x73e03 // 1010
	.word 0x73d30, 0x73e03 // 1011
	.word 0xbf203, 0x73d30 // 1100
	.word 0xbf130, 0x73d30 // 1101
	.word 0x73e03, 0x73d30 // 1110
	.word 0x73d30, 0x73d30 // 1111
	// background, foreground = 11, 10
	.word 0xbf203, 0xbf203 // 0000
	.word 0xbf230, 0xbf203 // 0001
	.word 0xb3e03, 0xbf203 // 0010
	.word 0xb3e30, 0xbf203 // 0011
	.word 0xbf203, 0xbf230 // 0100
	.word 0xbf230, 0xbf230 // 0101
	.word 0xb3e03, 0xbf230 // 0110
	.word 0xb3e30, 0xbf230 // 0111
	.word 0xbf203, 0xb3e03 // 1000
	.word 0xbf230, 0xb3e03 // 1001
	.word 0xb3e03, 0xb3e03 // 1010
	.word 0xb3e30, 0xb3e03 // 1011
	.word 0xbf203, 0xb3e30 // 1100
	.word 0xbf230, 0xb3e30 // 1101
	.word 0xb3e03, 0xb3e30 // 1110
	.word 0xb3e30, 0xb3e30 // 1111
	// background, foreground = 11, 11
	.word 0xbf203, 0xbf203 // 0000
	.word 0xbf203, 0xbf203 // 0001
	.word 0xbf203, 0xbf203 // 0010
	.word 0xbf203, 0xbf203 // 0011
	.word 0xbf203, 0xbf203 // 0100
	.word 0xbf203, 0xbf203 // 0101
	.word 0xbf203, 0xbf203 // 0110
	.word 0xbf203, 0xbf203 // 0111
	.word 0xbf203, 0xbf203 // 1000
	.word 0xbf203, 0xbf203 // 1001
	.word 0xbf203, 0xbf203 // 1010
	.word 0xbf203, 0xbf203 // 1011
	.word 0xbf203, 0xbf203 // 1100
	.word 0xbf203, 0xbf203 // 1101
	.word 0xbf203, 0xbf203 // 1110
	.word 0xbf203, 0xbf203 // 1111
