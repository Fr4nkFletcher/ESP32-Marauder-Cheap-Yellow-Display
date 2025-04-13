#ifndef _TMDS_ENCODE_FONT_2BPP_H
#define _TMDS_ENCODE_FONT_2BPP_H

#include "pico/types.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Render characters using an 8px-wide font and a per-character 2bpp
// foreground/background colour. This function is fast enough to run 3 times
// per scanline on one core, so RGB222 coloured text can be rendered (with
// separate R/G/B colour planes).
//
// charbuf: pointer to the row of characters (8 bits each) for the current
// scanline (byte-aligned)
//
// colourbuf: pointer to a list of 2bpp foreground/background colour pairs for
// each character (word-aligned, least-significant first within each word)
//
// font_line: pointer to list of 8 pixel bitmaps, each representing the
// intersection of a font character with the current scanline. (byte-aligned)

void tmds_encode_font_2bpp(const uint8_t *charbuf, const uint32_t	*colourbuf,
	uint32_t *tmdsbuf, uint n_pix, const uint8_t *font_line);

#if defined(__cplusplus)
}
#endif

#endif
