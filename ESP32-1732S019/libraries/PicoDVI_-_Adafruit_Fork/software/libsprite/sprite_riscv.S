// Functions for doing simple 2D graphics operations on a RGB scanline buffer.

#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

#include "sprite_asm_const.h"

#define POP2_OFFS   (SIO_INTERP0_POP_FULL_OFFSET   - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK0_OFFS  (SIO_INTERP0_PEEK_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK1_OFFS  (SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM0_OFFS (SIO_INTERP0_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_OFFS (SIO_INTERP0_ACCUM1_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define CTRL0_OFFS  (SIO_INTERP0_CTRL_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define INTERP1     (SIO_INTERP1_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)

#if defined(__riscv_c) || defined(__riscv_zca)
#define RISCV_HAVE_COMPRESSED_ISA 1
#endif

// ----------------------------------------------------------------------------
// Colour fill

// a0: dst
// a1: value
// a2: count

decl_func sprite_fill8
	// Slide for short fills
	li a3, 18
	bltu a3, a2, 2f
#ifndef RISCV_HAVE_COMPRESSED_ISA
#error "This address computation is wrong for non-RVC:"
#endif
	auipc a3, 0        // 32-bit instruction after address of auipc
	slli a2, a2, 2     // 16-bit instruction after address of auipc
	sub a3, a3, a2     // 16-bit instruction after address of auipc
	jr a3, 18 * 4 + 12 // 32-bit instruction after address of auipc
.align 2
	// With Zcb this is a mix of 16-bit and 32-bit instructions due to the
	// limited immediate size. Force 32-bit so we can do a computed branch.
.option push
.option norvc
	sb a1, 17(a0)
	sb a1, 16(a0)
	sb a1, 15(a0)
	sb a1, 14(a0)
	sb a1, 13(a0)
	sb a1, 12(a0)
	sb a1, 11(a0)
	sb a1, 10(a0)
	sb a1,  9(a0)
	sb a1,  8(a0)
	sb a1,  7(a0)
	sb a1,  6(a0)
	sb a1,  5(a0)
	sb a1,  4(a0)
	sb a1,  3(a0)
	sb a1,  2(a0)
	sb a1,  1(a0)
	sb a1,  0(a0)
.option pop
	ret
2:
	// Duplicate byte x4
	packh a1, a1, a1
	pack a1, a1, a1
	// Get a0 word-aligned:
	andi a3, a0, 0x1
	bnez a3, 1f
	sb a1, (a0)
	addi a0, a0, 1
	addi a2, a2, -1
1:
	andi a3, a0, 0x2
	bnez a3, 1f
	sh a1, (a0)
	addi a0, a0, 2
	addi a2, a2, -2
1:
	// Set up for main loop. Limit pointer at end - (loop body size)
	add a2, a2, a0
	addi a2, a2, -16

	// Fall straight into loop, because cases less than (loop body + max misalignment) are handled by slide
1:
	sw a1,  0(a0)
	sw a1,  4(a0)
	sw a1,  8(a0)
	sw a1, 12(a0)
	addi a0, a0, 16
	bgeu a2, a0, 1b

	// Main loop done, now tidy up the odds and ends. Note bits 3:0 of the
	// pointer difference are not affected by us subtracting 16 earlier.
	sub a2, a2, a0
	// No more than 15 bytes remaining -- first test bit 3 by shifting it to sign bit
	slli a2, a2, 28
	bgez a2, 1f
	sw a1, 0(a0)
	sw a1, 4(a0)
	addi a0, a0, 8
1:
	slli a2, a2, 1
	bgez a2, 1f
	sw a1, (a0)
	addi a0, a0, 4
1:
	slli a2, a2, 1
	bgez a2, 1f
	sh a1, (a0)
	addi a0, a0, 2
1:
	slli a2, a2, 1
	bgez a2, 1f
	sb a1, (a0)
1:
	ret

.p2align 2
decl_func sprite_fill16
	// Slide for short fills
	norvc_2a li a3, 16
	bltu a3, a2, 2f
#ifndef RISCV_HAVE_COMPRESSED_ISA
#error "This address computation is wrong for non-RVC:"
#endif
	auipc a3, 0        // 32-bit instruction after address of auipc
	slli a2, a2, 2     // 16-bit instruction after address of auipc
	sub a3, a3, a2     // 16-bit instruction after address of auipc
	jr a3, 16 * 4 + 12 // 32-bit instruction after address of auipc
.option push
.option norvc
	sh a1, 30(a0)
	sh a1, 28(a0)
	sh a1, 26(a0)
	sh a1, 24(a0)
	sh a1, 22(a0)
	sh a1, 20(a0)
	sh a1, 18(a0)
	sh a1, 16(a0)
	sh a1, 14(a0)
	sh a1, 12(a0)
	sh a1, 10(a0)
	sh a1,  8(a0)
	sh a1,  6(a0)
	sh a1,  4(a0)
	sh a1,  2(a0)
	sh a1,  0(a0)
.option pop
	ret
2:
	// Get word-aligned before main fill loop
	andi a3, a2, 0x2
	beqz a3, 1f
	sh a1, (a0)
	addi a0, a0, 2
	addi a2, a2, -1
1:
	// Set limit pointer at end - (loop body size)
	slli a2, a2, 1
	add a2, a2, a0
	addi a2, a2, -32
	pack a1, a1, a1
	// We can fall through because cases < 1 loop are handled by slide
1:
	sw a1,  0(a0)
	sw a1,  4(a0)
	sw a1,  8(a0)
	sw a1, 12(a0)
	sw a1, 16(a0)
	sw a1, 20(a0)
	sw a1, 24(a0)
	sw a1, 28(a0)
	addi a0, a0, 32
	bgeu a2, a0, 1b

	// Most of the work done, we have a few more to tidy up -- note bits 4:1
	// of the pointer difference are not affected by earlier subtraction of 32
	sub a2, a2, a0

	// Bit 4 becomes sign bit
	slli a2, a2, 27
	bgez a2, 1f
	sw a1,  0(a0)
	sw a1,  4(a0)
	sw a1,  8(a0)
	sw a1, 12(a0)
	addi a0, a0, 16
1:
	slli a2, a2, 1
	bgez a2, 1f
	sw a1,  0(a0)
	sw a1,  4(a0)
	addi a0, a0, 8
1:
	slli a2, a2, 1
	bgez a2, 1f
	sw a1,  0(a0)
	addi a0, a0, 4
1:
	slli a2, a2, 1
	bgez a2, 1f
	sh a1,  0(a0)
1:
	ret


// ----------------------------------------------------------------------------
// Non-AT sprite


// TODO 8-bit version not yet ported to RISC-V
#if 0
// Unrolled loop body with an initial computed branch.

// a0: dst
// a1: src
// a2: pixel count
decl_func sprite_blit8
	mov ip, a0
	lsrs a3, a2, #3
	lsls a3, #3
	eors a2, a3   // a2 = pixels % 8, a3 = pixels - pixels % 8

	add a0, a3
	add a1, a3

	adr a3, 2f
	lsls a2, #2
	subs a3, a2
	adds a3, #1 // thumb bit >:(
	bx a3

.align 2
1:
	subs a0, #8
	subs a1, #8
	ldrb a3, [a1, #7]
	strb a3, [a0, #7]
	ldrb a3, [a1, #6]
	strb a3, [a0, #6]
	ldrb a3, [a1, #5]
	strb a3, [a0, #5]
	ldrb a3, [a1, #4]
	strb a3, [a0, #4]
	ldrb a3, [a1, #3]
	strb a3, [a0, #3]
	ldrb a3, [a1, #2]
	strb a3, [a0, #2]
	ldrb a3, [a1, #1]
	strb a3, [a0, #1]
	ldrb a3, [a1, #0]
	strb a3, [a0, #0]
2:
	cmp a0, ip
	bhi 1b
	bx lr

.macro sprite_blit8_alpha_body n
	ldrb a3, [a1, #\n]
	lsrs a2, a3, #ALPHA_SHIFT_8BPP
	bcc 2f
	strb a3, [a0, #\n]
2:
.endm

// a0: dst
// a1: src
// a2: pixel count
decl_func sprite_blit8_alpha
	mov ip, a0
	lsrs a3, a2, #3
	lsls a3, #3
	eors a2, a3

	add a0, a3
	add a1, a3

	adr a3, 3f
	lsls a2, #3
	subs a3, a2
	adds a3, #1
	bx a3

.align 2
1:
	subs a0, #8
	subs a1, #8
	sprite_blit8_alpha_body 7
	sprite_blit8_alpha_body 6
	sprite_blit8_alpha_body 5
	sprite_blit8_alpha_body 4
	sprite_blit8_alpha_body 3
	sprite_blit8_alpha_body 2
	sprite_blit8_alpha_body 1
	sprite_blit8_alpha_body 0
3:
	cmp a0, ip
	bhi 1b
	bx lr

#endif

// Note this is the same ideal cycle count as lhu; lhu; sh; sh; but it reduces
// the number of memory accesses by 25%, so less bus contention
.macro storew_alignh rd ra offs
	sh \rd, \offs(\ra)
	srli \rd, \rd, 16
	sh \rd, \offs+2(\ra)
.endm

// a0: dst
// a1: src
// a2: pixel count
decl_func sprite_blit16
	// Force source pointer to be word-aligned
	andi a3, a1, 2
	beqz a3, 1f
	lhu a3, (a1)
	sh a3, (a0)
	addi a0, a0, 2
	addi a1, a1, 2
	addi a2, a2, -1
1:
	// Each loop is 8 pixels. Place limit pointer at 16 bytes before
	// end, loop until past it. There will be 0 to 7 pixels remaining.
	slli a2, a2, 1
	add a2, a2, a0
	addi a5, a2, -16
	// Early out:
	bltu a5, a0, 2f
1:
	lw a2, 0(a1)
	lw a3, 4(a1)
	storew_alignh a2, a0, 0
	storew_alignh a3, a0, 4
	lw a2, 8(a1)
	lw a3, 12(a1)
	storew_alignh a2, a0, 8
	storew_alignh a3, a0, 12
	addi a0, a0, 16
	addi a1, a1, 16
	bgeu a5, a0, 1b
2:
	sub a5, a5, a0
	// At least 4 pixels? (bit 3 -> sign bit)
	slli a5, a5, 28
	bgez a5, 1f
	lw a2, 0(a1)
	lw a3, 4(a1)
	storew_alignh a2, a0, 0
	storew_alignh a3, a0, 4
	addi a0, a0, 8
	addi a1, a1, 8
1:
	// At least 2 pixels?
	slli a5, a5, 1
	bgez a5, 1f
	lw a2, 0(a1)
	storew_alignh a2, a0, 0
	addi a0, a0, 4
	addi a1, a1, 4
1:
	// One more pixel?
	slli a5, a5, 1
	bgez a5, 1f
	lhu a3, (a1)
	sh a3, (a0)
1:
	ret

// dst: a0, src: a1, clobbers: a4-a7
.macro sprite_blit16_alpha_body_x2 n
	// Disable RVC to force 32-bit alignment of branch targets without adding
	// alignment nops (lhu/sh *may* be 16-bit if Zcb is enabled)
.option push
.option norvc
	// Interleave two loads to avoid load->shift dependency stall
	lhu a4, 4*\n(a1)
	lhu a5, 4*\n+2(a1)
	slli a6, a4, 32 - ALPHA_SHIFT_16BPP
	slli a7, a5, 32 - ALPHA_SHIFT_16BPP
	bgez a6, 3f
	sh a4, 4*\n(a0)
3:
	bgez a7, 3f
	sh a5, 4*\n+2(a0)
3:
.option pop
.endm

// a0: dst
// a1: src
// a2: pixel count
decl_func sprite_blit16_alpha
	// Not using the computed branch approach of the v6-M code as it doesn't
	// play nicely with the pairing of pixels used in the loop body here.
	slli a2, a2, 1
	add a2, a2, a0
	norvc_3a addi, a2, a2, -16
	bltu a2, a0, 2f
1:
	// 8 pixels per loop
	sprite_blit16_alpha_body_x2 0
	sprite_blit16_alpha_body_x2 1
	sprite_blit16_alpha_body_x2 2
	sprite_blit16_alpha_body_x2 3
	addi a0, a0, 16
	addi a1, a1, 16
	bgeu a2, a0, 1b
2:
	sub a2, a2, a0
	// At least 4 pixels? (bit 3 -> sign bit)
	slli a2, a2, 28
	bgez a2, 1f
	sprite_blit16_alpha_body_x2 0
	sprite_blit16_alpha_body_x2 1
	addi a0, a0, 8
	addi a1, a1, 8
1:
	// At least 2 pixels?
	norvc_3a slli, a2, a2, 1
	bgez a2, 1f
	sprite_blit16_alpha_body_x2 0
	addi a1, a1, 4
	addi a0, a0, 4
1:
	// One more pixel?
	slli a2, a2, 1
	bgez a2, 1f
	lhu a4, (a1)
	slli a6, a4, 32 - ALPHA_SHIFT_16BPP
	bgez a6, 1f
	sh a4, (a0)
1:
	ret
// ----------------------------------------------------------------------------
// Affine-transformed sprite (note these are just the inner loops -- INTERP0
// must be configured by the caller, which is presumably not written in asm)

// TODO not yet ported to RISC-V
#if 0
// r0: raster start pointer
// r1: raster span size (pixels)

.macro sprite_ablit8_loop_body n
	ldr r1, [r3, #CTRL0_OFFS]
	ldr r2, [r3, #POP2_OFFS]
	lsrs r1, #SIO_INTERP0_CTRL_LANE0_OVERF_LSB + 1
	bcs 2f
	ldrb r2, [r2]
	strb r2, [r0, #\n]
2:
.endm

decl_func sprite_ablit8_loop
	mov ip, r0

	lsrs r2, r1, #3
	lsls r2, #3
	eors r1, r2
	add r0, r2

	adr r2, 3f
	movs r3, #12 // Each (non-unrolled) loop body is 12 bytes
	muls r1, r3
	subs r2, r1
	adds r2, #1

	ldr r3, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	bx r2

.align 2
	nop
1:
	subs r0, #8
	sprite_ablit8_loop_body 7
	sprite_ablit8_loop_body 6
	sprite_ablit8_loop_body 5
	sprite_ablit8_loop_body 4
	sprite_ablit8_loop_body 3
	sprite_ablit8_loop_body 2
	sprite_ablit8_loop_body 1
	sprite_ablit8_loop_body 0
3:
	cmp r0, ip
	bne 1b
	bx lr



// As above but bit 5 is assumed to be an alpha bit (RAGB2132)

.macro sprite_ablit8_alpha_loop_body n
	ldr r1, [r3, #CTRL0_OFFS]
	ldr r2, [r3, #POP2_OFFS]
	lsrs r1, #SIO_INTERP0_CTRL_LANE0_OVERF_LSB + 1
	bcs 2f
	ldrb r2, [r2]
	lsrs r1, r2, #ALPHA_SHIFT_8BPP
	bcc 2f
	strb r2, [r0, #\n]
2:
.endm

decl_func sprite_ablit8_alpha_loop
	mov ip, r0
	ldr r3, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)

	lsrs r2, r1, #3
	lsls r2, #3
	eors r1, r2
	add r0, r2

	adr r2, 3f
	lsls r1, #4 // Each (non-unrolled) loop body is 16 bytes
	subs r2, r1
	adds r2, #1
	bx r2

.align 2
	nop
1:
	subs r0, #8
	sprite_ablit8_alpha_loop_body 7
	sprite_ablit8_alpha_loop_body 6
	sprite_ablit8_alpha_loop_body 5
	sprite_ablit8_alpha_loop_body 4
	sprite_ablit8_alpha_loop_body 3
	sprite_ablit8_alpha_loop_body 2
	sprite_ablit8_alpha_loop_body 1
	sprite_ablit8_alpha_loop_body 0
3:
	cmp r0, ip
	bhi 1b
	bx lr



.macro sprite_ablit16_loop_body n
	ldr r1, [r3, #CTRL0_OFFS]
	ldr r2, [r3, #POP2_OFFS]
	lsrs r1, #SIO_INTERP0_CTRL_LANE0_OVERF_LSB + 1
	bcs 2f
	ldrh r2, [r2]
	strh r2, [r0, #2*\n]
2:
.endm

decl_func sprite_ablit16_loop
	mov ip, r0

	lsrs r2, r1, #3
	lsls r2, #3
	eors r1, r2
	lsls r2, #1 // Each pixel is 2 bytes
	add r0, r2

	adr r2, 3f
	movs r3, #12 // Each (non-unrolled) loop body is 12 bytes
	muls r1, r3
	subs r2, r1
	adds r2, #1

	ldr r3, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	bx r2

.align 2
	nop
1:
	subs r0, #16
	sprite_ablit16_loop_body 7
	sprite_ablit16_loop_body 6
	sprite_ablit16_loop_body 5
	sprite_ablit16_loop_body 4
	sprite_ablit16_loop_body 3
	sprite_ablit16_loop_body 2
	sprite_ablit16_loop_body 1
	sprite_ablit16_loop_body 0
3:
	cmp r0, ip
	bne 1b
	bx lr

#endif

#define FIX_OVERF_CHECK 1

#ifndef RISCV_HAVE_COMPRESSED_ISA
#error "Address calculations are incorrect if not assembled with C extension"
#endif
.macro sprite_ablit16_alpha_loop_body n
	// Instructions which are only compressible under Zcb (e.g. lhu, sh) are
	// forced uncompressed, to get consistent size for address calculations.
	// This code should be exactly 24 bytes.

	// Bit 25 is OVERF, bit 24 is OVERF1, bits 31:26 are zero, so can test for
	// overflow by testing the uppermost byte of CTRL0 for nonzero.
#if !FIX_OVERF_CHECK
	norvc_2a lbu a1, CTRL0_OFFS+3(a5)
	lw a2, POP2_OFFS(a5)
	bnez a1, 2f
#else
	lw a1, ACCUM0_OFFS(a5)
	lw a3, ACCUM1_OFFS(a5)
	lw a2, POP2_OFFS(a5)
	srli a1, a1, 7 + 16
	bnez a1, 2f
	srli a3, a3, 7 + 16
	bnez a3, 2f
#endif
	norvc_2a lhu a2, (a2)
	// TODO dep stall on lhu, but it makes the OVERF case faster:
	slli a1, a2, 32 - ALPHA_SHIFT_16BPP
	bgez a1, 2f
	norvc_2a sh a2, 2*\n(a0)
2:
.endm

decl_func sprite_ablit16_alpha_loop
	mv a4, a0
	li a5, SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET

	// Split off pixels modulo 8
	andi a2, a1, 0x7
	sub a1, a1, a2
	// Pointer to beginning of endmost block of 8 pixels:
	sh1add a0, a1, a0

	// Compute branch into first loop, which has the modulo-8 pixels.
	// Each pixel takes 24 bytes of instructions.
#if !FIX_OVERF_CHECK
	slli a2, a2, 3
	sh1add a2, a2, a2
#else
	li a3, 30
	mul a2, a2, a3
#endif

	la a1, 3f
	sub a1, a1, a2
	jr a1

.align 2
1:
	norvc_3a addi a0, a0, -16
	sprite_ablit16_alpha_loop_body 7
	sprite_ablit16_alpha_loop_body 6
	sprite_ablit16_alpha_loop_body 5
	sprite_ablit16_alpha_loop_body 4
	sprite_ablit16_alpha_loop_body 3
	sprite_ablit16_alpha_loop_body 2
	sprite_ablit16_alpha_loop_body 1
	sprite_ablit16_alpha_loop_body 0
3:
	bltu a4, a0, 1b
	ret
