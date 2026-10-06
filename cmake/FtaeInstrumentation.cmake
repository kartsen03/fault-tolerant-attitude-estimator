# Optional instrumentation for the host build: sanitizers and coverage.
# Applied to every target so the tests and the code under test agree.

option(FTAE_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(FTAE_COVERAGE "Instrument for gcov line and branch coverage (GCC)" OFF)
option(FTAE_MCDC "Instrument for Clang source-based coverage with MC/DC (Clang 18+)" OFF)

if(FTAE_SANITIZE)
  set(_san -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  add_compile_options(${_san})
  add_link_options(${_san})
endif()

if(FTAE_COVERAGE)
  if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "FTAE_COVERAGE expects GCC (gcov); use FTAE_MCDC with Clang")
  endif()
  add_compile_options(--coverage -O0)
  add_link_options(--coverage)
endif()

if(FTAE_MCDC)
  if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang" OR CMAKE_C_COMPILER_VERSION VERSION_LESS 18)
    message(FATAL_ERROR "FTAE_MCDC needs Clang 18 or newer (-fcoverage-mcdc)")
  endif()
  set(_cov -fprofile-instr-generate -fcoverage-mapping)
  add_compile_options(${_cov} $<$<COMPILE_LANGUAGE:C>:-fcoverage-mcdc> -O0)
  add_link_options(${_cov})
endif()
