# RP2040 firmware: Pico SDK + FreeRTOS-Kernel (RP2040 port).
# Dependencies are fetched by tools/fetch_firmware_deps.sh; build with
# tools/build_firmware.sh.
cmake_minimum_required(VERSION 3.13...3.28)

get_filename_component(FTAE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(PICO_SDK_PATH "${FTAE_ROOT}/third_party/pico-sdk" CACHE PATH "Pico SDK location")
set(FREERTOS_KERNEL_PATH "${FTAE_ROOT}/third_party/FreeRTOS-Kernel" CACHE PATH "FreeRTOS-Kernel location")
set(PICO_BOARD pico CACHE STRING "Target board")

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")

project(ftae_firmware VERSION 0.1.0 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
pico_sdk_init()

set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config")
include("${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake")

# The same portable libraries as the host build, cross-compiled.
include("${FTAE_ROOT}/cmake/FtaeWarnings.cmake")
add_subdirectory("${FTAE_ROOT}/core" core)
add_subdirectory("${FTAE_ROOT}/hal" hal)  # interface only when cross-compiling
add_subdirectory("${FTAE_ROOT}/drivers" drivers)
add_subdirectory("${FTAE_ROOT}/firmware/app" app)

execute_process(
  COMMAND git -C "${FTAE_ROOT}" rev-parse --short=12 HEAD
  OUTPUT_VARIABLE FTAE_GIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT FTAE_GIT_SHA)
  set(FTAE_GIT_SHA "unknown")
endif()

add_executable(ftae_fw
  main.c
  "${FTAE_ROOT}/hal/rp2040/hal_rp2040.c")
target_include_directories(ftae_fw PRIVATE
  "${FTAE_ROOT}/hal/rp2040"
  "${CMAKE_CURRENT_LIST_DIR}/config")
target_compile_definitions(ftae_fw PRIVATE
  FTAE_VERSION="${PROJECT_VERSION}"
  FTAE_GIT_SHA="${FTAE_GIT_SHA}")
target_link_libraries(ftae_fw PRIVATE
  att_app att_drivers att_core
  pico_stdlib hardware_i2c hardware_uart
  FreeRTOS-Kernel)

# Platform glue gets -Werror too. The strict set is not applied here because
# the SDK and kernel sources compiled into this target are not written to it.
set_source_files_properties(main.c "${FTAE_ROOT}/hal/rp2040/hal_rp2040.c"
  PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")

pico_enable_stdio_uart(ftae_fw 1)
pico_enable_stdio_usb(ftae_fw 0)
pico_add_extra_outputs(ftae_fw)
