#ifndef _SPRITE_ASM_CONST
#define _SPRITE_ASM_CONST

// Put every function in its own ELF section, to permit linker GC
.macro decl_func name
.section .time_critical.\name, "ax"
.global \name
.p2align 2
#ifndef __riscv
.type \name,%function
.thumb_func
#endif
\name:
.endm

// Assume RGAB5515 (so alpha is bit 5)
// Note the alpha bit being in the same position as RAGB2132 is a coincidence.
// We are just stealing an LSB such that we can treat our alpha pixels in the
// same way as non-alpha pixels when encoding (and the co-opted channel LSB
// always ends up being set on alpha pixels, which is pretty harmless)

// Also note this is expressed as a right-shift into the carry flag (on Arm),
// so this is equal to the bit index of the alpha bit plus 1. On RISC-V it's
// idiomatic to shift up to the sign bit instead, so a left shift of 32 - x
// should be used instead of a right shift of x.

#define ALPHA_SHIFT_16BPP 6

// Assume RAGB2132 (so alpha is bit 5)

#define ALPHA_SHIFT_8BPP 6

#ifdef __riscv
// Macros for forcing individual instructions to be 32 bits, to maintain
// branch target alignment without adding NOPs
.macro norvc_1a instr, arg0
.option push
.option norvc
\instr \arg0
.option pop
.endm

.macro norvc_2a instr, arg0, arg1
.option push
.option norvc
\instr \arg0, \arg1
.option pop
.endm

.macro norvc_3a instr, arg0, arg1, arg2
.option push
.option norvc
\instr \arg0, \arg1, \arg2
.option pop
.endm
#endif

#endif
