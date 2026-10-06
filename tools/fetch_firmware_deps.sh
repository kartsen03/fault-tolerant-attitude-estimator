#!/usr/bin/env bash
# Fetch the pinned firmware dependencies into third_party/ (git-ignored).
# Shallow clones without submodules: the firmware uses UART stdio, so the
# SDK's TinyUSB/Wi-Fi submodules are not needed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/third_party"

PICO_SDK_TAG="2.3.1"
FREERTOS_KERNEL_TAG="V11.3.1"

clone() {
  local url="$1" tag="$2" dir="$3"
  if [ -d "${dir}/.git" ]; then
    echo "already present: ${dir}"
  else
    git -c advice.detachedHead=false clone --quiet --depth 1 --branch "${tag}" "${url}" "${dir}"
  fi
  echo "$(basename "${dir}") ${tag} @ $(git -C "${dir}" rev-parse HEAD)"
}

mkdir -p "${DEST}"
clone https://github.com/raspberrypi/pico-sdk.git "${PICO_SDK_TAG}" "${DEST}/pico-sdk"
clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git "${FREERTOS_KERNEL_TAG}" "${DEST}/FreeRTOS-Kernel"
