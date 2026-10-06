# Strict warning set for first-party C code (core, drivers, app, HAL).
# Shared by the host build and the Pico firmware build.

option(FTAE_WERROR "Treat warnings as errors in first-party C code" ON)

function(ftae_strict_warnings target)
  set(_warnings
      -Wall
      -Wextra
      -Wpedantic
      -Wshadow
      -Wconversion
      -Wsign-conversion
      -Wdouble-promotion
      -Wfloat-equal
      -Wcast-qual
      -Wcast-align
      -Wundef
      -Wstrict-prototypes
      -Wmissing-prototypes
      -Wold-style-definition
      -Wswitch-enum
      -Wswitch-default
      -Wformat=2
      -Wnull-dereference
      -Wvla
      -Wredundant-decls
      -Wwrite-strings)
  if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    list(APPEND _warnings
         -Wlogical-op
         -Wduplicated-cond
         -Wduplicated-branches
         -Wjump-misses-init)
  endif()
  if(FTAE_WERROR)
    list(APPEND _warnings -Werror)
  endif()
  target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:C>:${_warnings}>)
endfunction()
