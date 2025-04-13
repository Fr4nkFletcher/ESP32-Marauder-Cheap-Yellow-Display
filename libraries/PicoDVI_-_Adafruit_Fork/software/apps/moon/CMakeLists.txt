# First target: executable that uses regular software encode loop

add_executable(moon
	main.c
)

target_compile_options(moon PRIVATE -Wall)

target_compile_definitions(moon PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=3
	DVI_MONOCHROME_TMDS
	DVI_1BPP_BIT_REVERSE=0
)

target_link_libraries(moon
	pico_stdlib
	libdvi
)

pico_add_extra_outputs(moon)


# Second target (same source): executable that offloads TMDS encode to PIO + DMA

add_executable(moon_pio_encode
	main.c
)

target_compile_options(moon_pio_encode PRIVATE -Wall)

target_compile_definitions(moon_pio_encode PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=3
	DVI_MONOCHROME_TMDS
	USE_PIO_TMDS_ENCODE
)

target_link_libraries(moon_pio_encode
	pico_stdlib
	libdvi
)

pico_add_extra_outputs(moon_pio_encode)
