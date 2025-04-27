# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(mandelbrot
	main.c
)

target_compile_definitions(mandelbrot PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	)

target_link_libraries(mandelbrot
	pico_stdlib
	pico_multicore
	pico_util
	libdvi
	libsprite
)

# create map/bin/hex file etc.
pico_add_extra_outputs(mandelbrot)
