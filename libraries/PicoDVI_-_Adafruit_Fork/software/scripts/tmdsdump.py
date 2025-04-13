#!/usr/bin/env python3

from PIL import Image

dumpfiles = ("lane0.csv", "lane1.csv", "lane2.csv")

def loadcsv(csvfiles):
	records = []
	lines = tuple(list(open(f).readlines()) for f in csvfiles)
	for record in zip(*lines):
		records.append(tuple(int(lane.split(",")[1], 0) for lane in record))
	return records

ctrl_syms = {
	0b1101010100: 0, # 0x354
	0b0010101011: 1, # 0x0ab
	0b0101010100: 2, # 0x154
	0b1010101011: 3  # 0x2ab
}

def is_ctrl_sym(x):
	return x in ctrl_syms

def decode_data_sym(x):
	if x & 0x200:
		x ^= 0x2ff
	decode_trans = x & 0xff ^ (x << 1 & 0xff)
	return decode_trans & 0xff if (x & 0x100) else ~decode_trans & 0xff

def print_stream(stream):
	for t in stream:
		if any(is_ctrl_sym(s) for s in t):
			assert(all(is_ctrl_sym(s) for s in t))
			decode = ctrl_syms[t[2]]
			vsync = decode >> 1 & 1
			hsync = decode & 1
			print("VSYNC = {}, HSYNC = {}".format(vsync, hsync))
		else:
			print("Data: {:02x}, {:02x}, {:02x}".format(
				decode_data_sym(t[0]) >> 3, decode_data_sym(t[1]) >> 2, decode_data_sym(t[2]) >> 3))

def dump_frames(stream):
	current_line = []
	current_frame = []
	frame_ctr = 0
	last_ctrl = 0
	for t in stream:
		if is_ctrl_sym(t[2]):
			ctrl = ctrl_syms[t[2]]
			if (ctrl ^ last_ctrl) & 0x2 and len(current_frame) > 0:
				w, h = len(current_frame[0]), len(current_frame)
				print("Found frame {}. Size {} (w) x {} (h)".format(frame_ctr, w, h))
				img = Image.new('RGB', (w, h))
				for x in range(w):
					for y in range(h):
						img.putpixel((x, y), current_frame[y][x])
				img.save("frame{:02d}.png".format(frame_ctr))
				current_frame = []
				frame_ctr += 1
			elif len(current_line) > 0:
				current_frame.append(current_line)
				current_line = []
			last_ctrl = ctrl
		else:
			current_line.append(tuple(decode_data_sym(x) for x in t))


data = loadcsv(dumpfiles)
dump_frames(data)
