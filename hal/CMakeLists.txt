# HAL interface (header only). Every build needs it.
add_library(att_hal_api INTERFACE)
target_include_directories(att_hal_api INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Host simulator implementation: simulated clock plus a register-level
# MPU6050 model. Not built for the target; the RP2040 implementation is
# compiled by firmware/rp2040/CMakeLists.txt.
if(NOT CMAKE_CROSSCOMPILING)
  add_library(att_hal_host STATIC
    host/hal_host.c
    host/mpu6050_model.c)
  target_include_directories(att_hal_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/host)
  target_link_libraries(att_hal_host PUBLIC att_hal_api att_core)
  ftae_strict_warnings(att_hal_host)
endif()
