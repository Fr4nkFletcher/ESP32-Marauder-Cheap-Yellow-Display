#include "hardware/interp.h"
#include "tmds_encode.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

static const __unused uint32_t __scratch_x("tmds_table") tmds_table[] = {
#include "tmds_table.h"
};

// Fullres table is bandwidth-critical, so gets one copy for each scratch
// memory. There is a third copy which can go in flash, because it's just used
// to generate palette LUTs. The ones we don't use will get garbage collected
// during linking.
const __unused uint32_t __scratch_x("tmds_table_fullres_x") tmds_table_fullres_x[] = {
#include "tmds_table_fullres.h"
};

const __unused uint32_t __scratch_y("tmds_table_fullres_y") tmds_table_fullres_y[] = {
#include "tmds_table_fullres.h"
};

#if !DVI_USE_SIO_TMDS_ENCODER
// Configure an interpolator to extract a single colour channel from each of a pair
// of pixels, with the first pixel's lsb at pixel_lsb, and the pixels being
// pixel_width wide. Produce a LUT address for the first pixel's colour data on
// LANE0, and the second pixel's colour data on LANE1.
//
// Returns nonzero if the *_leftshift variant of the encoder loop must be used
// (needed for blue channel because I was a stubborn idiot and didn't put
// signed/bidirectional shift on interpolator, very slightly slower). The
// return value is the size of left shift required.

static int __not_in_flash_func(configure_interp_for_addrgen)(interp_hw_t *interp, uint channel_msb, uint channel_lsb, uint pixel_lsb, uint pixel_width, uint lut_index_width, const uint32_t *lutbase) {
	interp_config c;
	const uint index_shift = 2; // scaled lookup for 4-byte LUT entries

	int shift_channel_to_index = pixel_lsb + channel_msb - (lut_index_width - 1) - index_shift;
	int oops = 0;
#if PICO_RP2040
	if (shift_channel_to_index < 0) {
		// "It's ok we'll fix it in software"
		oops = -shift_channel_to_index;
		shift_channel_to_index = 0;
	}
#else
	// Now a right-rotate, not a right-shift
	shift_channel_to_index &= 0x1f;
#endif

	uint index_msb = index_shift + lut_index_width - 1;

	c = interp_default_config();
	interp_config_set_shift(&c, shift_channel_to_index);
	interp_config_set_mask(&c, index_msb - (channel_msb - channel_lsb), index_msb);
	interp_set_config(interp, 0, &c);

	c = interp_default_config();
	interp_config_set_shift(&c, pixel_width	+ shift_channel_to_index);
	interp_config_set_mask(&c, index_msb - (channel_msb - channel_lsb), index_msb);
	interp_config_set_cross_input(&c, true);
	interp_set_config(interp, 1, &c);

	interp->base[0] = (uint32_t)lutbase;
	interp->base[1] = (uint32_t)lutbase;

	return oops;
}

#else
// Encoding a single channel at a time is not the most efficient way to use
// this hardware, because it means we read the colour buffer multiple times,
// but it fits better with how things are done in software on RP2040.
static void __not_in_flash_func(configure_sio_tmds_for_single_channel)(uint channel_msb, uint channel_lsb, uint pixel_width, bool hdouble) {
	assert(channel_msb - channel_lsb <= 7); // 1 through 8 bits, inclusive
	sio_hw->tmds_ctrl =
		SIO_TMDS_CTRL_CLEAR_BALANCE_BITS |
		((channel_msb - channel_lsb) << SIO_TMDS_CTRL_L0_NBITS_LSB) |
		(((channel_msb - 7u) & 0xfu) << SIO_TMDS_CTRL_L0_ROT_LSB) |
		((1 + __builtin_ctz(pixel_width)) << SIO_TMDS_CTRL_PIX_SHIFT_LSB) |
		((uint)hdouble << SIO_TMDS_CTRL_PIX2_NOSHIFT_LSB);
}
#endif

// Extract up to 6 bits from a buffer of 16 bit pixels, and produce a buffer
// of TMDS symbols from this colour channel. Number of pixels must be even,
// pixel buffer must be word-aligned.

void __not_in_flash_func(tmds_encode_data_channel_16bpp)(const uint32_t *pixbuf, uint32_t *symbuf, size_t n_pix, uint channel_msb, uint channel_lsb) {
#if DVI_USE_SIO_TMDS_ENCODER
	configure_sio_tmds_for_single_channel(channel_msb, channel_lsb, 16, true);
#if DVI_SYMBOLS_PER_WORD == 1
	tmds_encode_sio_loop_peekpop_ratio4(pixbuf, symbuf, 2 * n_pix);
#else
	tmds_encode_sio_loop_poppop_ratio2(pixbuf, symbuf, 2 * n_pix);
#endif
#else
	interp_hw_save_t interp0_save;
	interp_save(interp0_hw, &interp0_save);
	int require_lshift = configure_interp_for_addrgen(interp0_hw, channel_msb, channel_lsb, 0, 16, 6, tmds_table);
#if PICO_RP2040
	if (require_lshift)
		tmds_encode_loop_16bpp_leftshift(pixbuf, symbuf, n_pix, require_lshift);
	else
		tmds_encode_loop_16bpp(pixbuf, symbuf, n_pix);
#else
	assert(!require_lshift); (void)require_lshift;
	tmds_encode_loop_16bpp(pixbuf, symbuf, n_pix);
#endif
	interp_restore(interp0_hw, &interp0_save);
#endif
}

// As above, but 8 bits per pixel, multiple of 4 pixels, and still word-aligned.
void __not_in_flash_func(tmds_encode_data_channel_8bpp)(const uint32_t *pixbuf, uint32_t *symbuf, size_t n_pix, uint channel_msb, uint channel_lsb) {
#if DVI_USE_SIO_TMDS_ENCODER
	configure_sio_tmds_for_single_channel(channel_msb, channel_lsb, 8, true);
#if DVI_SYMBOLS_PER_WORD == 1
	tmds_encode_sio_loop_peekpop_ratio8(pixbuf, symbuf, 2 * n_pix);
#else
	tmds_encode_sio_loop_poppop_ratio4(pixbuf, symbuf, 2 * n_pix);
#endif
#else
	interp_hw_save_t interp0_save, interp1_save;
	interp_save(interp0_hw, &interp0_save);
	interp_save(interp1_hw, &interp1_save);
	// Note that for 8bpp, some left shift is always required for pixel 0 (any
	// channel), which destroys some MSBs of pixel 3. To get around this, pixel
	// data sent to interp1 is *not left-shifted*
	int require_lshift = configure_interp_for_addrgen(interp0_hw, channel_msb, channel_lsb, 0, 8, 6, tmds_table);
	int lshift_upper = configure_interp_for_addrgen(interp1_hw, channel_msb, channel_lsb, 16, 8, 6, tmds_table);
	assert(!lshift_upper); (void)lshift_upper;
#if PICO_RP2040
	if (require_lshift)	
		tmds_encode_loop_8bpp_leftshift(pixbuf, symbuf, n_pix, require_lshift);
	else
		tmds_encode_loop_8bpp(pixbuf, symbuf, n_pix);
#else
	assert(!require_lshift); (void)require_lshift;
	tmds_encode_loop_8bpp(pixbuf, symbuf, n_pix);
#endif
	interp_restore(interp0_hw, &interp0_save);
	interp_restore(interp1_hw, &interp1_save);
#endif
}

// ----------------------------------------------------------------------------
// Code for full-resolution TMDS encode (barely possible, utterly impractical):

// Different scheme used for full res as the fun pixel-doubling DC balance
// trick doesn't work, so we need to actually do running disparity. ACCUM0 has
// pixel data, ACCUM1 has running disparity. INTERP0 is used to process even
// pixels, and INTERP1 for odd pixels. Note this means that even and odd
// symbols have their DC balance handled separately, which is not to spec.

#if !DVI_USE_SIO_TMDS_ENCODER
static int __not_in_flash_func(configure_interp_for_addrgen_fullres)(interp_hw_t *interp, uint channel_msb, uint channel_lsb, uint lut_index_width, const uint32_t *lutbase) {
	const uint index_shift = 2; // scaled lookup for 4-byte LUT entries

	int shift_channel_to_index = channel_msb - (lut_index_width - 1) - index_shift;
	int oops = 0;
#if PICO_RP2040
	if (shift_channel_to_index < 0) {
		// "It's ok we'll fix it in software"
		oops = -shift_channel_to_index;
		shift_channel_to_index = 0;
	}
#else
	// Now a right-rotate rather than right-shift
	shift_channel_to_index &= 0x1f;
#endif

	uint index_msb = index_shift + lut_index_width - 1;

	interp_config c;
	// Shift and mask colour channel to lower 6 bits of LUT index (note lut_index_width excludes disparity sign)
	c = interp_default_config();
	interp_config_set_shift(&c, shift_channel_to_index);
	interp_config_set_mask(&c, index_msb - (channel_msb - channel_lsb), index_msb);
	interp_set_config(interp, 0, &c);

	// Concatenate disparity (ACCUM1) sign onto the LUT index
	c = interp_default_config();
	interp_config_set_shift(&c, 30 - index_msb);
	interp_config_set_mask(&c, index_msb + 1, index_msb + 1);
	interp_set_config(interp, 1, &c);

	interp->base[2] = (uint32_t)lutbase;

	return oops;
}
#endif

void __not_in_flash_func(tmds_encode_data_channel_fullres_16bpp)(const uint32_t *pixbuf, uint32_t *symbuf, size_t n_pix, uint channel_msb, uint channel_lsb) {
#if DVI_USE_SIO_TMDS_ENCODER
	configure_sio_tmds_for_single_channel(channel_msb, channel_lsb, 16, false);
#if DVI_SYMBOLS_PER_WORD == 1
	tmds_encode_sio_loop_poppop_ratio2(pixbuf, symbuf, n_pix);
#else
	tmds_encode_sio_loop_poppop_ratio1(pixbuf, symbuf, n_pix);
#endif
#else
	uint core = get_core_num();
#if !TMDS_FULLRES_NO_INTERP_SAVE
	interp_hw_save_t interp0_save, interp1_save;
	interp_save(interp0_hw, &interp0_save);
	interp_save(interp1_hw, &interp1_save);
#endif

	// There is a copy of the inner loop and the LUT in both scratch X and
	// scratch Y memories. Use X on core 1 and Y on core 0 so the cores don't
	// tread on each other's toes too much.
	const uint32_t *lutbase = core ? tmds_table_fullres_x : tmds_table_fullres_y;
	int lshift_lower = configure_interp_for_addrgen_fullres(interp0_hw, channel_msb, channel_lsb, 6, lutbase);
	int lshift_upper = configure_interp_for_addrgen_fullres(interp1_hw, channel_msb + 16, channel_lsb + 16, 6, lutbase);
	assert(!lshift_upper); (void)lshift_upper;
	if (lshift_lower) {
		(core ?
			tmds_fullres_encode_loop_16bpp_leftshift_x :
			tmds_fullres_encode_loop_16bpp_leftshift_y
		)(pixbuf, symbuf, n_pix, lshift_lower);
	}
	else {
		(core ?
			tmds_fullres_encode_loop_16bpp_x :
			tmds_fullres_encode_loop_16bpp_y
		)(pixbuf, symbuf, n_pix);
	}
#if !TMDS_FULLRES_NO_INTERP_SAVE
	interp_restore(interp0_hw, &interp0_save);
	interp_restore(interp1_hw, &interp1_save);
#endif
#endif
}

static const int8_t imbalance_lookup[16] = { -4, -2, -2, 0, -2, 0, 0, 2, -2, 0, 0, 2, 0, 2, 2, 4 };

static inline int byte_imbalance(uint32_t x) {
	return imbalance_lookup[x >> 4] + imbalance_lookup[x & 0xF];
}

static void tmds_encode_symbols(uint8_t pixel, uint32_t* negative_balance_sym, uint32_t* positive_balance_sym) {
	int pixel_imbalance = byte_imbalance(pixel);
	uint32_t sym = pixel & 1;
	if (pixel_imbalance > 0 || (pixel_imbalance == 0 && sym == 0)) {
		for (int i = 0; i < 7; ++i) {
			sym |= (~((sym >> i) ^ (pixel >> (i + 1))) & 1) << (i + 1);
		}
	}
	else {
		for (int i = 0; i < 7; ++i) {
			sym |= ( ((sym >> i) ^ (pixel >> (i + 1))) & 1) << (i + 1);
		}
		sym |= 0x100;
	}

	int imbalance = byte_imbalance(sym & 0xFF);
	if (imbalance == 0) {
		if ((sym & 0x100) == 0) sym ^= 0x2ff;
		*positive_balance_sym = sym;
		*negative_balance_sym = sym;
		return;
	}
	else if (imbalance > 0) {
		*negative_balance_sym = (sym ^ 0x2ff) | (((-imbalance + imbalance_lookup[2 ^ (sym >> 8)] + 2) & 0x3F) << 26);
		*positive_balance_sym = sym | ((imbalance + imbalance_lookup[sym >> 8] + 2) << 26);
	}
	else {
		*negative_balance_sym = sym | (((imbalance + imbalance_lookup[sym >> 8] + 2) & 0x3F) << 26);
		*positive_balance_sym = (sym ^ 0x2ff) | ((-imbalance + imbalance_lookup[2 ^ (sym >> 8)] + 2) << 26);
	}
}

// This takes a 16-bit (RGB 565) colour palette and makes palettes of TMDS symbols suitable
// for performing fullres encode.
// The TMDS palette buffer should be 6 * n_palette words long.
// n_palette must be a power of 2 <= 256.
void tmds_setup_palette_symbols(const uint16_t *palette, uint32_t *tmds_palette, size_t n_palette) {
	uint32_t* tmds_palette_blue = tmds_palette;
	uint32_t* tmds_palette_green = tmds_palette + 2 * n_palette;
	uint32_t* tmds_palette_red = tmds_palette + 4 * n_palette;
	for (int i = 0; i < n_palette; ++i) {
		uint16_t blue = (palette[i] << 3) & 0xf8;
		uint16_t green = (palette[i] >> 3) & 0xfc;
		uint16_t red = (palette[i] >> 8) & 0xf8;
		tmds_encode_symbols(blue, &tmds_palette_blue[i], &tmds_palette_blue[i + n_palette]);
		tmds_encode_symbols(green, &tmds_palette_green[i], &tmds_palette_green[i + n_palette]);
		tmds_encode_symbols(red, &tmds_palette_red[i], &tmds_palette_red[i + n_palette]);
	}
}

// This takes a 24-bit (RGB 888) colour palette and makes palettes of TMDS symbols suitable
// for performing fullres encode.
// The TMDS palette buffer should be 6 * n_palette words long.
// n_palette must be a power of 2 <= 256.
void tmds_setup_palette24_symbols(const uint32_t *palette, uint32_t *tmds_palette, size_t n_palette) {
	uint32_t* tmds_palette_blue = tmds_palette;
	uint32_t* tmds_palette_green = tmds_palette + 2 * n_palette;
	uint32_t* tmds_palette_red = tmds_palette + 4 * n_palette;
	for (int i = 0; i < n_palette; ++i) {
		uint16_t blue = palette[i] & 0xff;
		uint16_t green = (palette[i] >> 8) & 0xff;
		uint16_t red = (palette[i] >> 16) & 0xff;
		tmds_encode_symbols(blue, &tmds_palette_blue[i], &tmds_palette_blue[i + n_palette]);
		tmds_encode_symbols(green, &tmds_palette_green[i], &tmds_palette_green[i + n_palette]);
		tmds_encode_symbols(red, &tmds_palette_red[i], &tmds_palette_red[i + n_palette]);
	}
}

// Encode palette data for all 3 channels.
// pixbuf is an array of n_pix 8-bit wide pixels containing palette values (32-bit word aligned)
// tmds_palette is a palette of TMDS symbols produced by tmds_setup_palette_symbols
// symbuf is 3*n_pix 32-bit words, this function writes the symbol values for each of the channels to it.
void __not_in_flash_func(tmds_encode_palette_data)(const uint32_t *pixbuf, const uint32_t *tmds_palette, uint32_t *symbuf, size_t n_pix, uint32_t palette_bits) {
	uint core = get_core_num();
#if !TMDS_FULLRES_NO_INTERP_SAVE
	interp_hw_save_t interp0_save, interp1_save;
	interp_save(interp0_hw, &interp0_save);
	interp_save(interp1_hw, &interp1_save);
#endif

	interp0_hw->base[2] = (uint32_t)tmds_palette;
	interp1_hw->base[2] = (uint32_t)tmds_palette;

	// Lane 0 on both interpolators masks the palette bits, starting at bit 2,
	// The second interpolator also shifts to read the 2nd or 4th byte of the word.
	interp0_hw->ctrl[0] =
		(2 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) |
		((palette_bits + 1) << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB);
	interp1_hw->ctrl[0] =
		(8 << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB) |
		(2 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) |
		((palette_bits + 1) << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB);

	// Lane 1 shifts and masks the sign bit into the right position to add to the symbol
	// table index to choose the negative disparity symbols if the sign is negative.
	const uint32_t ctrl_lane_1 =
		((31 - (palette_bits + 2)) << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB) |
		(palette_bits + 2) * ((1 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) | (1 << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB));
	interp0_hw->ctrl[1] = ctrl_lane_1;
	interp1_hw->ctrl[1] = ctrl_lane_1;

	if (core) {
		tmds_palette_encode_loop_x(pixbuf, symbuf, n_pix);

		interp0_hw->base[2] = (uint32_t)(tmds_palette + (2 << palette_bits));
		interp1_hw->base[2] = (uint32_t)(tmds_palette + (2 << palette_bits));
		tmds_palette_encode_loop_x(pixbuf, symbuf + (n_pix >> 1), n_pix);

		interp0_hw->base[2] = (uint32_t)(tmds_palette + (4 << palette_bits));
		interp1_hw->base[2] = (uint32_t)(tmds_palette + (4 << palette_bits));
		tmds_palette_encode_loop_x(pixbuf, symbuf + n_pix, n_pix);
	} else {
		tmds_palette_encode_loop_y(pixbuf, symbuf, n_pix);

		interp0_hw->base[2] = (uint32_t)(tmds_palette + (2 << palette_bits));
		interp1_hw->base[2] = (uint32_t)(tmds_palette + (2 << palette_bits));
		tmds_palette_encode_loop_y(pixbuf, symbuf + (n_pix >> 1), n_pix);

		interp0_hw->base[2] = (uint32_t)(tmds_palette + (4 << palette_bits));
		interp1_hw->base[2] = (uint32_t)(tmds_palette + (4 << palette_bits));
		tmds_palette_encode_loop_y(pixbuf, symbuf + n_pix, n_pix);
	}

#if !TMDS_FULLRES_NO_INTERP_SAVE
	interp_restore(interp0_hw, &interp0_save);
	interp_restore(interp1_hw, &interp1_save);
#endif
}
