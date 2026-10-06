cmake_minimum_required(VERSION 3.22)

project(fault_tolerant_attitude_estimator
  VERSION 0.1.0
  DESCRIPTION "Fault-tolerant IMU attitude estimator with SIL validation"
  LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FTAE_BUILD_TESTS "Build the GoogleTest suites" ON)

include(cmake/FtaeWarnings.cmake)
include(cmake/FtaeInstrumentation.cmake)

add_subdirectory(core)
add_subdirectory(hal)
add_subdirectory(drivers)

if(FTAE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
