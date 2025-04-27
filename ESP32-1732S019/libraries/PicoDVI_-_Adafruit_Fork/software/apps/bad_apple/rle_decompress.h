#ifndef _RLE_COMPRESS_H
#define _RLE_COMPRESS_H

#include <stdint.h>
#include <stddef.h>

void rle_to_tmds(const uint8_t *src, uint32_t *dst, size_t src_count);

extern uint32_t tmds_bw_pix0;
extern uint32_t tmds_bw_pix1;

#endif