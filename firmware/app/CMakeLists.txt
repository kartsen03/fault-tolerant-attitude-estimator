# Platform-independent application logic, shared by the RP2040 firmware
# and the host SIL runner.
add_library(att_app STATIC app.c telemetry.c)
target_include_directories(att_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(att_app PUBLIC att_core att_drivers att_hal_api)
ftae_strict_warnings(att_app)
