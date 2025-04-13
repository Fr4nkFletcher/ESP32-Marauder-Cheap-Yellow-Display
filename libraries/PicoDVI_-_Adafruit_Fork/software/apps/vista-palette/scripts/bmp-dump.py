from PIL import Image
import sys
import struct

bmp_in = Image.open("img.bmp")
if bmp_in is None:
    print(fname + ".bmp not found")
    sys.exit(2)

bin_out = open("imgdata.bin", "ab")

palette = bmp_in.getpalette()
if palette is None or len(palette) > 256 * 3:
    print("Expected 8bpp BMP")
    sys.exit(3)

num_colours = len(palette) // 3

data = bmp_in.getdata()

if len(data) != bmp_in.width * bmp_in.height:
    print("Expected 8bpp BMP")
    sys.exit(4)

if max(data) > num_colours - 1:
    print("Expected 8bpp BMP")
    sys.exit(4)

bin_out.write(struct.pack("<I", num_colours))
bin_out.write(struct.pack("<I", len(data)))

for i in range(num_colours):
    val = palette[3 * i] << 16
    val |= palette[3 * i + 1] << 8
    val |= palette[3 * i + 2]
    bin_out.write(struct.pack("<I", val))

for val in data:
    bin_out.write(struct.pack("=B", val))
