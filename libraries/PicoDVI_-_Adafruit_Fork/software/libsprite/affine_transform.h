#ifndef _AFFINE_TRANSFORM_H_
#define _AFFINE_TRANSFORM_H_

// Stolen from RISCBoy

#include <stdint.h>
#include "pico.h"

// Store unpacked affine transforms as signed 16.16 fixed point in the following order:
// a00, a01, b0,   a10, a11, b1
// i.e. the top two rows of the matrix
// [ a00 a01 b0 ]
// [ a01 a11 b1 ]
// [ 0   0   1  ]
// Then pack integers appropriately 

typedef int32_t affine_transform_t[6];
static const int32_t AF_ONE = 1 << 16;

static inline __attribute__((always_inline)) int32_t mul_fp1616(int32_t x, int32_t y)
{
	int64_t result = (int64_t)x * y;
	return result >> 16;
}

// result can not be == left or right
static inline void affine_mul(affine_transform_t result, const affine_transform_t left, const affine_transform_t right)
{
	result[0] = mul_fp1616(left[0], right[0]) + mul_fp1616(left[1], right[3]);
	result[1] = mul_fp1616(left[0], right[1]) + mul_fp1616(left[1], right[4]);
	result[2] = mul_fp1616(left[0], right[2]) + mul_fp1616(left[1], right[5]) + left[2];
	result[3] = mul_fp1616(left[3], right[0]) + mul_fp1616(left[4], right[3]);
	result[4] = mul_fp1616(left[3], right[1]) + mul_fp1616(left[4], right[4]);
	result[5] = mul_fp1616(left[3], right[2]) + mul_fp1616(left[4], right[5]) + left[5];
}

static inline void affine_copy(affine_transform_t dst, const affine_transform_t src)
{
	for (int i = 0; i < 6; ++i)
		dst[i] = src[i];
}

// User is describing a sequence of transformations from texture space to
// screen space, which are applied by premultiplying a column vector. However,
// hardware transforms *from* screenspace *to* texture space, so we want the
// inverse of the transform the user is building. Therefore our functions each
// produce the inverse of the requested transform, and we apply transforms by
// *post*-multiplication.
static inline void affine_identity(affine_transform_t current_trans)
{
	int32_t tmp[6] = {
		AF_ONE, 0,      0,
		0,      AF_ONE, 0
	};
	affine_copy(current_trans, tmp);
}

static inline void affine_translate(affine_transform_t current_trans, int32_t x, int32_t y)
{
	int32_t tmp[6];
	int32_t transform[6] = {
		AF_ONE, 0,      -AF_ONE * x,
		0,      AF_ONE, -AF_ONE * y
	};
	affine_mul(tmp, current_trans, transform);
	affine_copy(current_trans, tmp);
}

// TODO this is shit
static const int32_t __not_in_flash("atrans") sin_lookup_fp1616[256] = {
	0x0, 0x648, 0xc8f, 0x12d5, 0x1917, 0x1f56, 0x2590, 0x2bc4, 0x31f1, 0x3817,
	0x3e33, 0x4447, 0x4a50, 0x504d, 0x563e, 0x5c22, 0x61f7, 0x67bd, 0x6d74,
	0x7319, 0x78ad, 0x7e2e, 0x839c, 0x88f5, 0x8e39, 0x9368, 0x987f, 0x9d7f,
	0xa267, 0xa736, 0xabeb, 0xb085, 0xb504, 0xb968, 0xbdae, 0xc1d8, 0xc5e4,
	0xc9d1, 0xcd9f, 0xd14d, 0xd4db, 0xd848, 0xdb94, 0xdebe, 0xe1c5, 0xe4aa,
	0xe76b, 0xea09, 0xec83, 0xeed8, 0xf109, 0xf314, 0xf4fa, 0xf6ba, 0xf853,
	0xf9c7, 0xfb14, 0xfc3b, 0xfd3a, 0xfe13, 0xfec4, 0xff4e, 0xffb1, 0xffec,
	0x10000, 0xffec, 0xffb1, 0xff4e, 0xfec4, 0xfe13, 0xfd3a, 0xfc3b, 0xfb14,
	0xf9c7, 0xf853, 0xf6ba, 0xf4fa, 0xf314, 0xf109, 0xeed8, 0xec83, 0xea09,
	0xe76b, 0xe4aa, 0xe1c5, 0xdebe, 0xdb94, 0xd848, 0xd4db, 0xd14d, 0xcd9f,
	0xc9d1, 0xc5e4, 0xc1d8, 0xbdae, 0xb968, 0xb504, 0xb085, 0xabeb, 0xa736,
	0xa267, 0x9d7f, 0x987f, 0x9368, 0x8e39, 0x88f5, 0x839c, 0x7e2e, 0x78ad,
	0x7319, 0x6d74, 0x67bd, 0x61f7, 0x5c22, 0x563e, 0x504d, 0x4a50, 0x4447,
	0x3e33, 0x3817, 0x31f1, 0x2bc4, 0x2590, 0x1f56, 0x1917, 0x12d5, 0xc8f, 0x648,
	0x0, 0xfffff9b8, 0xfffff371, 0xffffed2b, 0xffffe6e9, 0xffffe0aa, 0xffffda70,
	0xffffd43c, 0xffffce0f, 0xffffc7e9, 0xffffc1cd, 0xffffbbb9, 0xffffb5b0,
	0xffffafb3, 0xffffa9c2, 0xffffa3de, 0xffff9e09, 0xffff9843, 0xffff928c,
	0xffff8ce7, 0xffff8753, 0xffff81d2, 0xffff7c64, 0xffff770b, 0xffff71c7,
	0xffff6c98, 0xffff6781, 0xffff6281, 0xffff5d99, 0xffff58ca, 0xffff5415,
	0xffff4f7b, 0xffff4afc, 0xffff4698, 0xffff4252, 0xffff3e28, 0xffff3a1c,
	0xffff362f, 0xffff3261, 0xffff2eb3, 0xffff2b25, 0xffff27b8, 0xffff246c,
	0xffff2142, 0xffff1e3b, 0xffff1b56, 0xffff1895, 0xffff15f7, 0xffff137d,
	0xffff1128, 0xffff0ef7, 0xffff0cec, 0xffff0b06, 0xffff0946, 0xffff07ad,
	0xffff0639, 0xffff04ec, 0xffff03c5, 0xffff02c6, 0xffff01ed, 0xffff013c,
	0xffff00b2, 0xffff004f, 0xffff0014, 0xffff0000, 0xffff0014, 0xffff004f,
	0xffff00b2, 0xffff013c, 0xffff01ed, 0xffff02c6, 0xffff03c5, 0xffff04ec,
	0xffff0639, 0xffff07ad, 0xffff0946, 0xffff0b06, 0xffff0cec, 0xffff0ef7,
	0xffff1128, 0xffff137d, 0xffff15f7, 0xffff1895, 0xffff1b56, 0xffff1e3b,
	0xffff2142, 0xffff246c, 0xffff27b8, 0xffff2b25, 0xffff2eb3, 0xffff3261,
	0xffff362f, 0xffff3a1c, 0xffff3e28, 0xffff4252, 0xffff4698, 0xffff4afc,
	0xffff4f7b, 0xffff5415, 0xffff58ca, 0xffff5d99, 0xffff6281, 0xffff6781,
	0xffff6c98, 0xffff71c7, 0xffff770b, 0xffff7c64, 0xffff81d2, 0xffff8753,
	0xffff8ce7, 0xffff928c, 0xffff9843, 0xffff9e09, 0xffffa3de, 0xffffa9c2,
	0xffffafb3, 0xffffb5b0, 0xffffbbb9, 0xffffc1cd, 0xffffc7e9, 0xffffce0f,
	0xffffd43c, 0xffffda70, 0xffffe0aa, 0xffffe6e9, 0xffffed2b, 0xfffff371,
	0xfffff9b8
};

static inline int32_t sin_fp1616(uint8_t theta)
{
	return sin_lookup_fp1616[theta];
}

static inline int32_t cos_fp1616(uint8_t theta)
{
	return sin_lookup_fp1616[(theta + 64) & 0xff];
}

// Appears as a counterclockwise rotation (when viewed from texture space to screen space)
// Units of angle are 256 = one turn
static inline void affine_rotate(affine_transform_t current_trans, uint8_t theta)
{
	int32_t tmp[6];
	int32_t transform[6] = {
		cos_fp1616(theta), -sin_fp1616(theta), 0,
		sin_fp1616(theta),  cos_fp1616(theta), 0
	};
	affine_mul(tmp, current_trans, transform);
	affine_copy(current_trans, tmp);
}

static inline void affine_scale(affine_transform_t current_trans, int32_t sx, int32_t sy) {
	int32_t sx_inv = ((int64_t)AF_ONE * AF_ONE) / sx;
	int32_t sy_inv = ((int64_t)AF_ONE * AF_ONE) / sy;
	int32_t tmp[6];
	int32_t transform[6] = {
		sx_inv, 0,      0,
		0,      sy_inv, 0
	};
	affine_mul(tmp, current_trans, transform);
	affine_copy(current_trans, tmp);
}

#endif // _AFFINE_TRANSFORM_H_
