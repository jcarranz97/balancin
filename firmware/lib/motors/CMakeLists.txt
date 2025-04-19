add_library(motors
    motors.cpp
)

target_include_directories(motors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(motors
	pico_stdlib
	hardware_pwm
)
