# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(dht_logging
	main.c
	dht.c
)

target_compile_options(dht_logging PRIVATE -Wall)

target_compile_definitions(dht_logging PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=3
	DVI_1BPP_BIT_REVERSE=0
	)


target_link_libraries(dht_logging
	pico_stdlib
	pico_multicore
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(dht_logging)
