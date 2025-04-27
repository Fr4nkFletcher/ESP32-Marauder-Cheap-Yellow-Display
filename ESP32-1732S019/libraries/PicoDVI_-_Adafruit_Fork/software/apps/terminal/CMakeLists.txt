# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(terminal
	main.c
)

# TODO this should work ok with DVI_N_TMDS_BUFFERS=2 (perhaps need to
# rearrange some pushes/pops) and also as we are monochrome the buffers are 3x
# as big as they need to be 
target_compile_definitions(terminal PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=3
	DVI_MONOCHROME_TMDS=1
	DVI_1BPP_BIT_REVERSE=0
	)

target_link_libraries(terminal
	pico_stdlib
	pico_multicore
	pico_util
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(terminal)
