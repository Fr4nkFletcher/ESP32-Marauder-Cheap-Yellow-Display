#!/usr/bin/env python3

# We avoid TMDS encode on the M0+ by holding the following sequences as
# prebalanced symbol pairs:
#
# - Black, black
# - Black, white
# - White, black
# - White, white
#
# A sequence of these pairs can encode any black and white image, so we use
# this byte encoding:
#
# 7      0
# abnnnnnn
#
# where a, b are the colours of the two pixels in the pair, and n is a repeat
# count between 1 and 64. We can expand this into TMDS scanlines very quickly
# on the M0+ and, because everything stays byte-aligned, it's amenable to
# further compression.

import sys
from PIL import Image

def fmt_byte(a, b, n):
	assert(n >= 1 and n <= 64)
	return a << 7 | b << 6 | (n - 1 & 0x3f)

# row is iterable of (bool, bool)
def pack_row(row):
	current = (False, False)
	current_len = 0
	while True:
		try:
			pix = next(row)
		except StopIteration:
			if current_len > 0:
				yield fmt_byte(*current, current_len)
			break
		if pix == current or current_len == 0:
			current = pix
			current_len += 1
		if pix != current:
			yield fmt_byte(*current, current_len)
			current = pix
			current_len = 1
		if current_len >= 64:
			yield fmt_byte(*current, current_len)
			current_len = 0

def pack_image(img):
	assert(img.width % 2 == 0)
	assert(img.mode == "RGB")
	bimg = img.tobytes()
	w = img.width
	for y in range(img.height):
		row = bytes(pack_row(
			(bimg[3 * (x + w * y)] >= 0x80, bimg[3 * (x + 1 + w * y)] >= 0x80)
			for x in range(0, img.width, 2)))
		yield bytes((len(row),))
		yield row

if __name__ == "__main__":
	img = Image.open(sys.argv[1])
	open(sys.argv[2], "wb").write(b"".join(pack_image(img)))
