add_executable(vista
	main.c
)

# All code and data must be RAM-resident so we can stream continuously from
# flash using direct SSI DMA, which would trample on XIP.
pico_set_binary_type(vista copy_to_ram)

target_compile_definitions(vista PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=5
	DVI_SYMBOLS_PER_WORD=1
	)

if (PICO_RP2040)
	pico_define_boot_stage2(vista_boot2 ${PICO_SDK_PATH}/src/rp2040/boot_stage2/boot2_w25q080.S)
	pico_set_boot_stage2(vista vista_boot2)
	target_compile_definitions(vista_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
else ()
	target_compile_definitions(vista PRIVATE
		PICO_EMBED_XIP_SETUP=1
		PICO_BOOT_STAGE2_CHOOSE_W25Q080=1
		PICO_FLASH_SPI_CLKDIV=2
		PICO_FLASH_SPI_RXDELAY=3
		)
endif()

target_compile_definitions(vista PRIVATE PICO_STACK_SIZE=0x200)

target_link_libraries(vista
	pico_stdlib
	pico_multicore
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(vista)
