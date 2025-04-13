// Functions for doing simple 2D graphics operations on a RGB scanline buffer.

#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

#include "sprite_asm_const.h"

#define POP2_OFFS (SIO_INTERP0_POP_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define CTRL0_OFFS (SIO_INTERP0_CTRL_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define INTERP1 (SIO_INTERP1_ACCUM0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)

.syntax unified
.cpu cortex-m0plus
.thumb

// ----------------------------------------------------------------------------
// Colour fill

// r0: dst
// r1: value
// r2: count

decl_func sprite_fill8
	// Slide for short fills
	cmp r2, #18
	bhi 2f
	adr r3, 1f
	lsls r2, #1
	subs r3, r2
	adds r3, #1 // thumb bit
	bx r3
.align 2
	strb r1, [r0, #17]
	strb r1, [r0, #16]
	strb r1, [r0, #15]
	strb r1, [r0, #14]
	strb r1, [r0, #13]
	strb r1, [r0, #12]
	strb r1, [r0, #11]
	strb r1, [r0, #10]
	strb r1, [r0, #9]
	strb r1, [r0, #8]
	strb r1, [r0, #7]
	strb r1, [r0, #6]
	strb r1, [r0, #5]
	strb r1, [r0, #4]
	strb r1, [r0, #3]
	strb r1, [r0, #2]
	strb r1, [r0, #1]
	strb r1, [r0, #0]
1:
	bx lr
2:
	lsls r3, r1, #8
	orrs r1, r3
	lsls r3, r1, #16
	orrs r1, r3
	// Get r0 word-aligned:
	lsrs r3, r0, #1
	bcc 1f
	strb r1, [r0]
	adds r0, #1
	subs r2, #1
1:
	lsrs r3, r0, #2
	bcc 1f
	strh r1, [r0]
	adds r0, #2
	subs r2, #2
1:
	// Set up for main loop. Limit pointer at end - (loop body size - 1)
	push {r4}
	adds r2, r0
	subs r2, #15
	mov ip, r2
	mov r2, r1
	mov r3, r1
	mov r4, r1

	// Fall straight into loop, because cases less than (loop body + max misalignment) are handled by slide
1:
	stmia r0!, {r1, r2, r3, r4}
	cmp r0, ip
	blo 1b

	// Main loop done, now tidy up the odds and ends
	mov r4, ip
	subs r4, r0
	adds r4, #15
	// No more than 15 bytes remaining -- first test bit 3
	lsls r4, #29
	bcc 1f
	stmia r0!, {r1, r2}
1:
	lsls r4, #1
	bcc 1f
	stmia r0!, {r1}
1:
	lsls r4, #1
	bcc 1f
	strh r1, [r0]
	adds r0, #2
1:
	lsls r4, #1
	bcc 1f
	strb r1, [r0]
1:
	pop {r4}
	bx lr


decl_func sprite_fill16
	// Slide for short fills
	cmp r2, #15
	bhi 2f
	adr r3, 1f
	lsls r2, #1
	subs r3, r2
	adds r3, #1
	bx r3
.align 2
	strh r1, [r0, #30]
	strh r1, [r0, #28]
	strh r1, [r0, #26]
	strh r1, [r0, #24]
	strh r1, [r0, #22]
	strh r1, [r0, #20]
	strh r1, [r0, #18]
	strh r1, [r0, #16]
	strh r1, [r0, #14]
	strh r1, [r0, #12]
	strh r1, [r0, #10]
	strh r1, [r0, #8]
	strh r1, [r0, #6]
	strh r1, [r0, #4]
	strh r1, [r0, #2]
	strh r1, [r0, #0]
1:
	bx lr
2:
	push {r4, r5, r6, r7, lr}
	// Get word-aligned before main fill loop
	lsrs r3, r2, #2
	bcc 1f
	strh r1, [r0]
	adds r0, #2
	subs r2, #1
1:
	// Set limit pointer at end - (loop body size - 1)
	lsls r2, #1
	adds r2, r0
	subs r2, #26
	mov ip, r2

	lsls r2, r1, #16
	orrs r1, r2
	mov r2, r1
	mov r3, r1
	mov r4, r1
	mov r5, r1
	mov r6, r1
	mov r7, r1
	// We can fall through because cases < 1 loop are handled by slide
1:
	stmia r0!, {r1, r2, r3, r4, r5, r6, r7} // wheeeeeeeeeee
	cmp r0, ip
	blo 1b

	// Most of the work done, we have a few more to tidy up
	movs r2, #26
	add r2, ip
	subs r2, r0

	lsls r2, #28
	bcc 1f
	stmia r0!, {r4, r5, r6, r7}
1:
	lsls r2, #1
	bcc 1f
	stmia r0!, {r4, r5}
1:
	lsls r2, #1
	bcc 1f
	stmia r0!, {r4}
1:
	lsls r2, #1
	bcc 1f
	strh r4, [r0]
1:
	pop {r4, r5, r6, r7, pc}

// ----------------------------------------------------------------------------
// Non-AT sprite

// r0: dst
// r1: src
// r2: pixel count
//

// Unrolled loop body with an initial computed branch.

decl_func sprite_blit8
	mov ip, r0
	lsrs r3, r2, #3
	lsls r3, #3
	eors r2, r3   // r2 = pixels % 8, r3 = pixels - pixels % 8

	add r0, r3
	add r1, r3

	adr r3, 2f
	lsls r2, #2
	subs r3, r2
	adds r3, #1 // thumb bit >:(
	bx r3

.align 2
1:
	subs r0, #8
	subs r1, #8
	ldrb r3, [r1, #7]
	strb r3, [r0, #7]
	ldrb r3, [r1, #6]
	strb r3, [r0, #6]
	ldrb r3, [r1, #5]
	strb r3, [r0, #5]
	ldrb r3, [r1, #4]
	strb r3, [r0, #4]
	ldrb r3, [r1, #3]
	strb r3, [r0, #3]
	ldrb r3, [r1, #2]
	strb r3, [r0, #2]
	ldrb r3, [r1, #1]
	strb r3, [r0, #1]
	ldrb r3, [r1, #0]
	strb r3, [r0, #0]
2:
	cmp r0, ip
	bhi 1b
	bx lr

.macro sprite_blit8_alpha_body n
	ldrb r3, [r1, #\n]
	lsrs r2, r3, #ALPHA_SHIFT_8BPP
	bcc 2f
	strb r3, [r0, #\n]
2:
.endm

decl_func sprite_blit8_alpha
	mov ip, r0
	lsrs r3, r2, #3
	lsls r3, #3
	eors r2, r3

	add r0, r3
	add r1, r3

	adr r3, 3f
	lsls r2, #3
	subs r3, r2
	adds r3, #1
	bx r3

.align 2
1:
	subs r0, #8
	subs r1, #8
	sprite_blit8_alpha_body 7
	sprite_blit8_alpha_body 6
	sprite_blit8_alpha_body 5
	sprite_blit8_alpha_body 4
	sprite_blit8_alpha_body 3
	sprite_blit8_alpha_body 2
	sprite_blit8_alpha_body 1
	sprite_blit8_alpha_body 0
3:
	cmp r0, ip
	bhi 1b
	bx lr


.macro storew_alignh rd ra offs
	strh \rd, [\ra, #\offs]
	lsrs \rd, #16
	strh \rd, [\ra, #\offs + 2]
.endm

decl_func sprite_blit16
	// Force source pointer to be word-aligned
	lsrs r3, r1, #2
	bcc 1f
	ldrh r3, [r1]
	strh r3, [r0]
	adds r0, #2
	adds r1, #2
	subs r2, #1
1:
	// Each loop is 8 pixels. Place limit pointer at 16 bytes before
	// end, loop until past it. There will be 0 to 7 pixels remaining.
	lsls r2, #1
	adds r2, r0
	subs r2, #16
	mov ip, r2
	b 2f
1:
	ldmia r1!, {r2, r3}
	storew_alignh r2, r0, 0
	storew_alignh r3, r0, 4
	ldmia r1!, {r2, r3}
	storew_alignh r2, r0, 8
	storew_alignh r3, r0, 12
	adds r0, #16
2:
	cmp r0, ip
	bls 1b

	mov r2, ip
	subs r2, r0
	// At least 4 pixels?
	lsls r2, #29
	bcc 1f
	ldmia r1!, {r3}
	storew_alignh r3, r0, 0
	ldmia r1!, {r3}
	storew_alignh r3, r0, 4
	adds r0, #8
1:
	// At least 2 pixels?
	lsls r2, #1
	bcc 1f
	ldmia r1!, {r3}
	storew_alignh r3, r0, 0
	adds r0, #4
1:
	// One more pixel?
	lsls r2, #1
	bcc 1f
	ldrh r3, [r1]
	strh r3, [r0]
1:
	bx lr

.macro sprite_blit16_alpha_body n
	ldrh r3, [r1, #2*\n]
	lsrs r2, r3, #ALPHA_SHIFT_16BPP
	bcc 2f
	strh r3, [r0, #2*\n]
2:
.endm

decl_func sprite_blit16_alpha
	mov ip, r0
	lsrs r3, r2, #3
	lsls r3, #3
	eors r2, r3

	lsls r3, #1
	add r0, r3
	add r1, r3

	adr r3, 3f
	lsls r2, #3
	subs r3, r2
	adds r3, #1
	bx r3

.align 2
1:
	subs r0, #16
	subs r1, #16
	sprite_blit16_alpha_body 7
	sprite_blit16_alpha_body 6
	sprite_blit16_alpha_body 5
	sprite_blit16_alpha_body 4
	sprite_blit16_alpha_body 3
	sprite_blit16_alpha_body 2
	sprite_blit16_alpha_body 1
	sprite_blit16_alpha_body 0
3:
	cmp r0, ip
	bhi 1b
	bx lr


// ----------------------------------------------------------------------------
// Affine-transformed sprite (note these are just the inner loops -- INTERP0
// must be configured by the caller, which is presumably not written in asm)

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



.macro sprite_ablit16_alpha_loop_body n
	ldr r1, [r3, #CTRL0_OFFS]
	ldr r2, [r3, #POP2_OFFS]
	lsrs r1, #SIO_INTERP0_CTRL_LANE0_OVERF_LSB + 1
	bcs 2f
	ldrh r2, [r2]
	lsrs r1, r2, #ALPHA_SHIFT_16BPP
	bcc 2f
	strh r2, [r0, #2*\n]
2:
.endm

decl_func sprite_ablit16_alpha_loop
	mov ip, r0
	ldr r3, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)

	lsrs r2, r1, #3
	lsls r2, #3
	eors r1, r2
	lsls r2, #1 // Each pixel is 2 bytes
	add r0, r2

	adr r2, 3f
	lsls r1, #4 // Each (non-unrolled) loop body is 16 bytes
	subs r2, r1
	adds r2, #1
	bx r2

.align 2
	nop
1:
	subs r0, #16
	sprite_ablit16_alpha_loop_body 7
	sprite_ablit16_alpha_loop_body 6
	sprite_ablit16_alpha_loop_body 5
	sprite_ablit16_alpha_loop_body 4
	sprite_ablit16_alpha_loop_body 3
	sprite_ablit16_alpha_loop_body 2
	sprite_ablit16_alpha_loop_body 1
	sprite_ablit16_alpha_loop_body 0
3:
	cmp r0, ip
	bhi 1b
	bx lr
