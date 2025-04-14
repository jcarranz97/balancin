add_library(I2Cdev
    I2Cdev.cpp
)

target_include_directories(I2Cdev PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(I2Cdev
	pico_stdlib
	hardware_i2c
)
