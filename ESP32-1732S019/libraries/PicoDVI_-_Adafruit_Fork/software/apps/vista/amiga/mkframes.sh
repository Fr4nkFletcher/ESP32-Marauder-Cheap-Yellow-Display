#!/bin/bash
set -ex

rm -rf raw
mkdir raw
ffmpeg -i src.mp4 -t 1 -filter:v "crop=1440:1080:240:0,fps=fps=20" -s 640x480 raw/frame%03d.bmp

for frame in $(find raw -name "*.bmp" | sort)
do
	../../../scripts/packtiles -sdf rgb565 ${frame} ${frame}.bin
done

cat $(find raw -name "*.bin" | sort) > raw/pack.bin
uf2conv -f pico -b 0x1003c000 raw/pack.bin -o amiga_data.uf2
