# MPU6050 driver. Calls only the HAL API; the HAL implementation is chosen
# by whatever links the final executable (target, host simulator, or mock).
add_library(att_drivers STATIC src/mpu6050.c)
target_include_directories(att_drivers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(att_drivers PUBLIC att_core att_hal_api)
ftae_strict_warnings(att_drivers)
