add_library(libsprite INTERFACE)

target_sources(libsprite INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/affine_transform.h
	${CMAKE_CURRENT_LIST_DIR}/sprite_asm_const.h
	${CMAKE_CURRENT_LIST_DIR}/sprite.c
	${CMAKE_CURRENT_LIST_DIR}/sprite.h
	${CMAKE_CURRENT_LIST_DIR}/tile.c
	${CMAKE_CURRENT_LIST_DIR}/tile.h
	)

if (PICO_RISCV)
	target_sources(libsprite INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/sprite_riscv.S
		${CMAKE_CURRENT_LIST_DIR}/tile_riscv.S
		)
else ()
	target_sources(libsprite INTERFACE
		${CMAKE_CURRENT_LIST_DIR}/sprite_armv6m.S
		${CMAKE_CURRENT_LIST_DIR}/tile_armv6m.S
		)
endif()


target_include_directories(libsprite INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(libsprite INTERFACE pico_base_headers hardware_interp)
