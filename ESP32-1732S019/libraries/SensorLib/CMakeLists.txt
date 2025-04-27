set(src_dirs ./src
             ./src/touch
             ./src/platform)
set(include_dirs ./src
                ./src/REG
                ./src/touch
                ./src/platform)
idf_component_register(SRC_DIRS ${src_dirs} 
                        INCLUDE_DIRS ${include_dirs}
                        REQUIRES esp_timer esp_driver_gpio driver)
