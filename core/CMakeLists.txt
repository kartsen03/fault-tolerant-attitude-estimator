# Portable attitude core: no hardware dependencies, no dynamic memory.
# Built unchanged for the host (tests, SIL) and for the RP2040 firmware.

add_library(att_core STATIC
  src/att_math.c
  src/comp_filter.c)

target_include_directories(att_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

ftae_strict_warnings(att_core)

# libm: sinf/cosf/atan2f/sqrtf/fmodf (glibc on the host, newlib on the target)
target_link_libraries(att_core PUBLIC m)
