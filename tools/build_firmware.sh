#!/usr/bin/env bash
# Cross-compile the RP2040 firmware. Run tools/fetch_firmware_deps.sh first.
# Usage: tools/build_firmware.sh [build-dir]   (default: build/rp2040)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-${ROOT}/build/rp2040}"

cmake -S "${ROOT}/firmware/rp2040" -B "${BUILD}" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD}"
arm-none-eabi-size "${BUILD}/ftae_fw.elf"
