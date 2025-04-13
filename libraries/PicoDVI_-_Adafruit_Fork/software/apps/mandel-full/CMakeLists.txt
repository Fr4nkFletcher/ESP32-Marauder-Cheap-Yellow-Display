# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(mandel-full
	main.c
  mandelbrot.c
)

target_compile_definitions(mandel-full PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=8
	DVI_SYMBOLS_PER_WORD=2
	)

target_compile_definitions(mandel-full PRIVATE PICO_STACK_SIZE=0x400)


target_link_libraries(mandel-full
	pico_stdlib
	pico_multicore
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(mandel-full)
