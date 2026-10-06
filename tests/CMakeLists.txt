# GoogleTest suites. Prefer the system package (Ubuntu: libgtest-dev);
# fall back to a pinned download when it is missing.
find_package(GTest 1.14 CONFIG QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)

# Unit tests: portable core, plus the driver against a scripted mock HAL.
add_executable(ftae_unit_tests
  core/test_att_math.cpp
  core/test_comp_filter.cpp
  core/test_estimator.cpp
  drivers/test_mpu6050.cpp
  mocks/hal_mock.cpp)
target_include_directories(ftae_unit_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/core/src  # white-box access to internal helpers
  ${CMAKE_CURRENT_SOURCE_DIR}/support
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
target_link_libraries(ftae_unit_tests PRIVATE att_drivers att_core GTest::gtest_main)
target_compile_options(ftae_unit_tests PRIVATE -Wall -Wextra -Werror)
gtest_discover_tests(ftae_unit_tests DISCOVERY_TIMEOUT 30)
