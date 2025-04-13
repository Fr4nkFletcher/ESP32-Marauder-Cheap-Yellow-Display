// Copyright (C) Michael Bell 2021

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/interp.h"
#include "hardware/dma.h"

#include "mandelbrot.h"

// Cycle checking parameters
#define MAX_CYCLE_LEN 8          // Must be power of 2
#define MIN_CYCLE_CHECK_ITER 32  // Must be multiple of max cycle len
#define CYCLE_TOLERANCE (1<<18)

// Fixed point with 6 bits to the left of the point.
// Range [-32,32) with precision 2^-26
typedef int32_t fixed_pt_t;

#define ESCAPE_SQUARE (4<<26)

static inline fixed_pt_t mul(fixed_pt_t a, fixed_pt_t b)
{
  int32_t ah = a >> 13;
  int32_t al = a & 0x1fff;
  int32_t bh = b >> 13;
  int32_t bl = b & 0x1fff;

  // Ignore al * bl as contribution to final result is only the carry.
  fixed_pt_t r = ((ah * bl) + (al * bh)) >> 13;
  r += ah * bh;
  return r;
}

// a * b * 2
static inline fixed_pt_t mul2(fixed_pt_t a, fixed_pt_t b)
{
#if 0
  int32_t ah = a >> 12;
  int32_t al = a & 0xfff;
  int32_t bh = b >> 13;
  int32_t bl = b & 0x1fff;

  interp0->accum[0] = ah * bl;
  interp0->accum[1] = al * bh;
  interp0->base[2] = ah * bh;
  return interp0->peek[2];
#else
  int32_t ah = a >> 12;
  int32_t al = (a & 0xfff) << 1;
  int32_t bh = b >> 13;
  int32_t bl = b & 0x1fff;

  fixed_pt_t r = ((ah * bl) + (al * bh)) >> 13;
  r += ah * bh;
  return r;
#endif
}

static inline fixed_pt_t square(fixed_pt_t a) {
  int32_t ah = a >> 13;
  int32_t al = a & 0x1fff;

  return ((ah * al) >> 12) + (ah * ah);
}

fixed_pt_t make_fixed(int32_t x) {
  return x << 26;
}

fixed_pt_t make_fixedf(float x) {
  return (int32_t)(x * (67108864.f));
}

void mandel_init()
{
  // Not curently used
  interp_config cfg = interp_default_config();
  interp_config_set_add_raw(&cfg, false);
  interp_config_set_shift(&cfg, 13);
  interp_config_set_mask(&cfg, 0, 31 - 13);
  interp_config_set_signed(&cfg, true);
  interp_set_config(interp0, 0, &cfg);
  interp_config_set_shift(&cfg, 12);
  interp_config_set_mask(&cfg, 0, 31 - 12);
  interp_set_config(interp0, 1, &cfg);
}

void init_fractal(FractalBuffer* f)
{
  f->done = false;
  f->min_iter = f->max_iter - 1;
  f->iminx = make_fixedf(f->minx);
  f->imaxx = make_fixedf(f->maxx);
  f->iminy = make_fixedf(f->miny);
  f->imaxy = make_fixedf(f->maxy);
  f->incx = (f->imaxx - f->iminx) / (f->cols - 1);
  f->incy = (f->imaxy - f->iminy) / (f->rows - 1);
  f->count_inside = 0;
  f->ipos = 0;
  f->jpos = 0;
  f->iend = f->rows - 1;
  f->jend = f->cols - 1;
}

static inline void generate_one(FractalBuffer* f, fixed_pt_t x0, fixed_pt_t y0, uint8_t* buffptr)
{
  fixed_pt_t x = x0;
  fixed_pt_t y = y0;

  uint16_t k = 1;
  for (; k < f->max_iter; ++k) {
    fixed_pt_t x_square = square(x);
    fixed_pt_t y_square = square(y);
    if (x_square + y_square > ESCAPE_SQUARE) break;

    fixed_pt_t nextx = x_square - y_square + x0;
    y = mul2(x,y) + y0;
    x = nextx;
  }
  if (k == f->max_iter) {
    *buffptr = 0;
    f->count_inside++;
  } else {
    if (k > f->iter_offset) k -= f->iter_offset;
    else k = 1;
    *buffptr = k;
    if (f->min_iter > k) f->min_iter = k;
  }
}

static inline void generate_one_cycle_check(FractalBuffer* f, fixed_pt_t x0, fixed_pt_t y0, uint8_t* buffptr)
{
  fixed_pt_t x = x0;
  fixed_pt_t y = y0;
  fixed_pt_t oldx = 0, oldy = 0;

  uint16_t k = 1;
  for (; k < f->max_iter; ++k) {
    fixed_pt_t x_square = square(x);
    fixed_pt_t y_square = square(y);
    if (x_square + y_square > ESCAPE_SQUARE) break;

    if (k >= MIN_CYCLE_CHECK_ITER) {
      if ((k & (MAX_CYCLE_LEN - 1)) == 0) {
        oldx = x - CYCLE_TOLERANCE;
        oldy = y - CYCLE_TOLERANCE;
      }
      else
      {
        if ((uint32_t)(x - oldx) < (2*CYCLE_TOLERANCE) && (uint32_t)(y - oldy) < (2*CYCLE_TOLERANCE)) {
          // Found a cycle
          k = f->max_iter;
          break;
        }
      }
    }

    fixed_pt_t nextx = x_square - y_square + x0;
    y = mul2(x,y) + y0;
    x = nextx;
  }
  if (k == f->max_iter) {
    *buffptr = 0;
    f->count_inside++;
  } else {
    if (k > f->iter_offset) k -= f->iter_offset;
    else k = 1;
    *buffptr = k;
    if (f->min_iter > k) f->min_iter = k;
  }
}

void generate_fractal(FractalBuffer* f)
{
  uint8_t* buffptr = f->buff;

  fixed_pt_t y0 = f->iminy;
  int16_t i = 0;
  for (; i < f->iend; ++i, y0 += f->incy) {
    fixed_pt_t x0 = f->iminx;
    for (int16_t j = 0; j < f->cols; ++j, x0 += f->incx) {
      if (f->use_cycle_check) generate_one_cycle_check(f, x0, y0, buffptr++);
      else generate_one(f, x0, y0, buffptr++);
    }
  }

  fixed_pt_t x0 = f->iminx;
  for (int16_t j = 0; j < f->jend && i == f->iend; ++j, x0 += f->incx) {
    if (f->use_cycle_check) generate_one_cycle_check(f, x0, y0, buffptr++);
    else generate_one(f, x0, y0, buffptr++);
  }

  f->done = true;
}

void generate_one_forward(FractalBuffer* f)
{
  if (f->done) return;

  uint8_t* buffptr = f->buff + f->ipos * f->cols + f->jpos;

  fixed_pt_t y0 = f->iminy + f->ipos * f->incy;
  fixed_pt_t x0 = f->iminx + f->jpos * f->incx;
  if (f->use_cycle_check) generate_one_cycle_check(f, x0, y0, buffptr);
  else generate_one(f, x0, y0, buffptr);

  if (++f->jpos == f->cols)
  {
    f->jpos = 0;
    if (++f->ipos > f->iend) f->done = true;
  }
}

void generate_steal_one(FractalBuffer* f)
{
  if (f->done) {
    return;
  }

  uint8_t* buffptr = f->buff + f->iend * f->cols + f->jend;

  fixed_pt_t y0 = f->iminy + f->iend * f->incy;
  fixed_pt_t x0 = f->iminx + f->jend * f->incx;
  if (f->use_cycle_check) generate_one_cycle_check(f, x0, y0, buffptr);
  else generate_one(f, x0, y0, buffptr);

  if (f->jend-- == 0) {
    f->jend = f->cols - 1;
    if (--f->iend < f->ipos) f->done = true;
  }
}
