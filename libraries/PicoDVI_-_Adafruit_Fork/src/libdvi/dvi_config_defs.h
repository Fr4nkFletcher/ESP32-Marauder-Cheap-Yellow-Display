#ifndef _DVI_CONFIG_DEFS_H
#define _DVI_CONFIG_DEFS_H

// Compile-time configuration definitions for libdvi. This file provides
// defaults -- you can override using a board header, or setting compile
// definitions directly from the commandline (e.g. using CMake
// target_compile_definitions())

// Pull in base headers to make sure board definitions override the
// definitions provided here. Note this file is included in asm and C.
#include "hardware/platform_defs.h"
#include "pico/config.h"

// ----------------------------------------------------------------------------
// General DVI defines

// How many times to output the same TMDS buffer before recyling it onto the
// free queue. Pixels are repeated vertically if this is >1.
#ifndef DVI_VERTICAL_REPEAT
#define DVI_VERTICAL_REPEAT 2
#endif

// Number of TMDS buffers to allocate (malloc()) in DVI init. You can set this
// to 0 if you want to allocate your own (e.g. if you want static buffers)
#ifndef DVI_N_TMDS_BUFFERS
#define DVI_N_TMDS_BUFFERS 3
#endif

// If 1, replace the DVI serialiser with a 10n1 UART (1 start bit, 10 data
// bits, 1 stop bit) so the stream can be dumped and analysed easily.
#ifndef DVI_SERIAL_DEBUG
#define DVI_SERIAL_DEBUG 0
#endif

// If 1, the same TMDS symbols are sent to all 3 lanes during the horizontal
// active period. This means only monochrome colour is available, but the TMDS
// buffers are 3 times smaller as a result, and the performance requirements
// for encode are also cut by 3.
#ifndef DVI_MONOCHROME_TMDS
#define DVI_MONOCHROME_TMDS 0
#endif

// By default, we assume each 32-bit word written to a PIO FIFO contains 2x
// 10-bit TMDS symbols, concatenated into the lower 20 bits, least-significant
// first. This is convenient if you are generating two or more pixels at once,
// e.g. using the pixel-doubling TMDS encode. You can change this value to 1
// (so each word contains 1 symbol) for e.g. full resolution RGB encode. Note
// that this value needs to divide the DVI horizontal timings, so is limited
// to 1 or 2.
#ifndef DVI_SYMBOLS_PER_WORD
#define DVI_SYMBOLS_PER_WORD 2
#endif

#if DVI_SYMBOLS_PER_WORD != 1 && DVI_SYMBOLS_PER_WORD !=2
#error "Unsupported value for DVI_SYMBOLS_PER_WORD"
#endif

// ----------------------------------------------------------------------------
// Pixel component layout

// By default we go R, G, B from MSB -> LSB. Override to e.g. swap RGB <-> BGR

// Default 8bpp layout: RGB332, {r[1:0], g[2:0], b[1:0]}

#ifndef DVI_8BPP_RED_MSB
#define DVI_8BPP_RED_MSB 7
#endif

#ifndef DVI_8BPP_RED_LSB
#define DVI_8BPP_RED_LSB 5
#endif

#ifndef DVI_8BPP_GREEN_MSB
#define DVI_8BPP_GREEN_MSB 4
#endif

#ifndef DVI_8BPP_GREEN_LSB
#define DVI_8BPP_GREEN_LSB 2
#endif

#ifndef DVI_8BPP_BLUE_MSB
#define DVI_8BPP_BLUE_MSB 1
#endif

#ifndef DVI_8BPP_BLUE_LSB
#define DVI_8BPP_BLUE_LSB 0
#endif

// Default 16bpp layout: RGB565, {r[4:0], g[5:0], b[4:0]}

#ifndef DVI_16BPP_RED_MSB
#define DVI_16BPP_RED_MSB 15
#endif

#ifndef DVI_16BPP_RED_LSB
#define DVI_16BPP_RED_LSB 11
#endif

#ifndef DVI_16BPP_GREEN_MSB
#define DVI_16BPP_GREEN_MSB 10
#endif

#ifndef DVI_16BPP_GREEN_LSB
#define DVI_16BPP_GREEN_LSB 5
#endif

#ifndef DVI_16BPP_BLUE_MSB
#define DVI_16BPP_BLUE_MSB 4
#endif

#ifndef DVI_16BPP_BLUE_LSB
#define DVI_16BPP_BLUE_LSB 0
#endif

// Default 1bpp layout: bitwise little-endian, i.e. least significant bit of
// each word is the first (leftmost) of a block of 32 pixels.

// If 1, reverse the order of pixels within each byte. Order of bytes within
// each word is still little-endian.
#ifndef DVI_1BPP_BIT_REVERSE
#define DVI_1BPP_BIT_REVERSE 1 // Adafruit_GFX GFXcanvas1 requires this 1
#endif

// ----------------------------------------------------------------------------
// TMDS encode controls

// Number of TMDS loop bodies between branches. cmp + branch costs 3 cycles,
// so you can easily save 10% of encode time by bumping this. Note that body
// will *already* produce multiple pixels, and total symbols per iteration
// must cleanly divide symbols per scanline, else the loop won't terminate.
// Point gun away from foot.
#ifndef TMDS_ENCODE_UNROLL
#define TMDS_ENCODE_UNROLL 1
#endif

// If 1, don't save/restore the interpolators on full-resolution TMDS encode.
// Speed hack. The TMDS code uses both interpolators, for each of the 3 data
// channels, so this define avoids 6 save/restores per scanline.
#ifndef TMDS_FULLRES_NO_INTERP_SAVE
#define TMDS_FULLRES_NO_INTERP_SAVE 0
#endif

// If 1, don't DC-balance the output of full resolution encode. Hilariously
// noncompliant, but Dell Ultrasharp -- the honey badger of computer monitors
// -- does not seem to mind (it helps that we DC-couple). Another speed hack,
// useful when you are trying to get everything else up to speed.
#ifndef TMDS_FULLRES_NO_DC_BALANCE
#define TMDS_FULLRES_NO_DC_BALANCE 0
#endif

#endif
