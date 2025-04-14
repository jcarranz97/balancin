add_library(MPU6050
    MPU6050.cpp
)

target_include_directories(MPU6050 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link it to I2Cdev
target_link_libraries(MPU6050 PUBLIC I2Cdev)
