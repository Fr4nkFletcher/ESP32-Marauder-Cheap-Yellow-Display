add_executable(colour_terminal
	main.c
	tmds_encode_font_2bpp.S
	tmds_encode_font_2bpp.h
)

target_compile_definitions(colour_terminal PRIVATE
	DVI_VERTICAL_REPEAT=1
	)

# We have a lot in SRAM4 (particularly TMDS LUT) but don't need much stack on
# core 1. Probably even 256 bytes would be fine.
target_compile_definitions(colour_terminal PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	PICO_CORE1_STACK_SIZE=0x200
	)

target_link_libraries(colour_terminal
	pico_stdlib
	pico_multicore
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(colour_terminal)
