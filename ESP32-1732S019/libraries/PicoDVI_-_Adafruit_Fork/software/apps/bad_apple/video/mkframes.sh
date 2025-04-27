#!/bin/bash
set -ex

rm -rf raw rle pack.bin pack.uf2

mkdir raw
ffmpeg \
	-ss 00:00:05.5 -to 00:00:12.5 \
	-i src.mkv \
	-f lavfi -i color=gray:s=1920x1080 \
	-f lavfi -i color=black:s=1920x1080 \
	-f lavfi -i color=white:s=1920x1080 \
	-filter_complex "threshold,fps=30,crop=1440:1080:240:0,scale=960x720" \
	raw/frame%04d.png

mkdir rle
make -f packframes.mk -j$(nproc)

cat $(find rle -name "*.bin" | sort) > pack.bin

uf2conv -f rp2040 -b 0x10010000 pack.bin -o pack.uf2
