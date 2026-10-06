#!/usr/bin/env bash
# Fail if a heap allocator is linked into the firmware image. The firmware
# allocates everything statically (FreeRTOS with static allocation only and
# no heap_N.c), and section garbage collection drops any allocator code that
# nothing calls, so these symbols must not appear.
set -euo pipefail

ELF="${1:?usage: check_no_heap.sh firmware.elf}"
NM="${NM:-arm-none-eabi-nm}"

if "${NM}" "${ELF}" | grep -E ' (malloc|free|calloc|realloc|_malloc_r|_free_r|_calloc_r|_realloc_r|pvPortMalloc|vPortFree)$'; then
  echo "FAIL: heap allocator symbols linked into ${ELF}" >&2
  exit 1
fi
echo "OK: no heap allocator symbols in ${ELF}"
