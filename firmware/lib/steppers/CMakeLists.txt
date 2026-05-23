add_library(steppers
    steppers.cpp
)

target_include_directories(steppers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(steppers
	pico_stdlib
	hardware_pwm
)
