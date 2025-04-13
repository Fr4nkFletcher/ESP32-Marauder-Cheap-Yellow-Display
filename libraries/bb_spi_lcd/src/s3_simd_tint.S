//
// ESP32-S3 SIMD optimized code
// Written by Larry Bank
// Copyright (c) 2025 BitBank Software, Inc.
// Project started March 7, 2025
//
#if defined (ARDUINO_ARCH_ESP32) && !defined(NO_SIMD)
#if __has_include ("dsps_fft2r_platform.h")
#include "dsps_fft2r_platform.h"
#if (dsps_fft2r_sc16_aes3_enabled == 1)
	.text
	.align 4

    .type   s3_masked_tint_be, @function
    .global s3_masked_tint_be

# Alpha blend a RGB565 color to the masked pixels
# The alpha value can be from 0 to 32
# The pixel count must be a multiple of 8
# Ideally it should take less than 10 clocks per pixel
# This code should be much simpler, but the S3 SIMD limitations
# cause extra thinking and work. For example, the only right shift
# available is 32-bits and arithmetic (copies sign bit)
# this messes up manipulation of 16-bit pixels (see below)
#
#                                        A2            A3            A4                   A5               A6               A7                A8
# Call as void s3_masked_tint_be(uint16_t *pDest, uint16_t *Src, uint16_t *pMask, uint16_t tintColor, uint32_t count, uint8_t alpha, const uint16_t *pMasks);
  s3_masked_tint_be:
    entry    a1,32
    l32i.n   a8,a1,32         # 7th parameter is passed on the stack

# make the alpha value as a pair of 16-bit values for later
  slli a9,a7,16
  or a9,a9,a7                 # now we have the alpha in 16-bit slots

  ee.movi.32.q q6,a9,0        # set up alpha value in 8 slots of Q6
  ee.movi.32.q q6,a9,1
  ee.movi.32.q q6,a9,2
  ee.movi.32.q q6,a9,3

  movi.n    a9,32           # inverted alpha for destination
  sub a7,a9,a7
  slli a9,a7,16             # prepare it for 16-bit slots
  or a9,a9,a7
  ee.movi.32.q q5,a9,0      # set up inverted alpha in 8 slots of Q5
  ee.movi.32.q q5,a9,1
  ee.movi.32.q q5,a9,2
  ee.movi.32.q q5,a9,3

# set up the tint color in q2
  slli a9,a5,16
  or a9,a9,a5
  ee.movi.32.q q2,a9,0     # copy to all slots of Q2
  ee.movi.32.q q2,a9,1
  ee.movi.32.q q2,a9,2
  ee.movi.32.q q2,a9,3

  movi.n a10,0               # prepare shift amounts
  movi.n a11,5

.tint_loop_be:

  wsr.sar    a10            # SAR (shift amount register) = 0

  ee.vld.128.ip  q1,a3,0    # load 8 RGB565 src pixels into q1
  mv.qr q7,q1               # big endian data, need to swap the byte order
  ee.vunzip.8 q7,q1         # swap the byte order to be big-endian
  ee.vzip.8 q1,q7

  ee.vldbc.16.ip q0,a8,2    # load the blue mask into all 8 slots
  ee.andq q3,q1,q0          # apply the mask to all source pixels
  ee.andq q4,q2,q0          # apply to the tint color
  ee.vmul.u16 q3,q3,q5      # multiply source pixels by source alpha
  ee.vmul.u16 q4,q4,q6      # multiply dest pixels by dest alpha
  ee.vadds.s16 q7,q3,q4     # combine src+dest blue
  wsr.sar a11               # set up right shift by 5
  ee.vsr.32 q7,q7           # shift blue back to normal position
  ee.andq q7,q7,q0

  ee.vldbc.16.ip q0,a8,2    # load the green mask
  ee.andq q3,q1,q0
  ee.andq q4,q2,q0
  ee.vmul.u16 q3,q3,q5      # mult green by src+dest alpha
  ee.vmul.u16 q4,q4,q6      # leave the right shift of 5
  ee.vadds.s16 q4,q3,q4     # combine src+dest green
  ee.andq q4,q4,q0
  ee.orq q7,q7,q4           # combine green+blue

  ee.vldbc.16.ip q0,a8,2    # load red mask for arith shift problem
  ee.vsr.32 q1,q1           # shift red down 5 bits to not overflow
  ee.vsr.32 q4,q2
  ee.andq q1,q1,q0          # mask at the new position
  ee.andq q4,q4,q0
  ee.vmul.u16 q1,q1,q5      # mult red by src+dest alpha
  ee.vmul.u16 q4,q4,q6
  ee.vadds.s16 q1,q1,q4     # combine src+dst red
  ee.vsl.32 q1,q1           # now shift left 5 to fix the position
  ee.vldbc.16 q0,a8         # load final RGB mask for proper position of R
  ee.andq q1,q1,q0
  ee.orq q1,q7,q1           # combine R + G + B

# we now have 16-bit slots with the results we want
  mv.qr q7,q1               # reverse byte order from little to big endian
  ee.vunzip.8 q7,q1
  ee.vzip.8 q1,q7

  ee.vld.128.ip  q7,a4,16   # load 8 mask pixels into q7
  ee.vld.128.ip  q3,a3,16   # re-load the 8 source pixels into q3
  ee.andq q1,q1,q7          # preserve the processed pixels where mask=FFFF
  ee.notq q7,q7             # invert the mask
  ee.andq q3,q3,q7          # get source pixels that were not part of the mask
  ee.orq q1,q1,q3           # combine for final output
  ee.vst.128.ip q1,a2,16    # store 8 finished destination pixels
  addi.n a8,a8,-6           # reset color masks pointer
  addi.n a6,a6,-8           # decrement pixel count by 8
  bnez.n a6,.tint_loop_be

  # return value of 0
    retw.n

#endif // dsps_fft2r_sc16_aes3_enabled
#endif // __has_include
#endif // ESP32
