set(TARGET_NAME robot)

add_executable(${TARGET_NAME}
    src/main.cpp
    )
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include/
    )

target_link_libraries(robot
    PRIVATE
    MPU6050  # This will pull I2Cdev as well
)

target_link_libraries(${TARGET_NAME} PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    pico_stdlib
    hardware_i2c
    MPU6050
    motors
    )
# Enable Serial over USB
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(${TARGET_NAME} PRIVATE
        pico_cyw43_arch_none
    )
endif()
target_compile_definitions(${TARGET_NAME} PRIVATE
    configNUMBER_OF_CORES=2
    )
pico_add_extra_outputs(${TARGET_NAME})
