RAW=$(sort $(wildcard raw/*))

.PHONY: all

all: $(patsubst raw/%.png,rle/%.bin,$(RAW))

rle/%.bin: raw/%.png
	./rle_compress.py $< $@
