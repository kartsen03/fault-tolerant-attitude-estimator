#!/usr/bin/env bash
# One-time setup of the Linux toolchain used for host builds, sanitizers,
# coverage (including MC/DC), static analysis and the Pico firmware build.
# Tested on Ubuntu 24.04 (WSL2). Needs sudo for apt.
set -euo pipefail

sudo apt-get update

# The cmake package was left half-installed on this machine
# ("Could not find CMAKE_ROOT"); reinstalling restores its data files.
sudo apt-get install -y --reinstall cmake cmake-data

sudo apt-get install -y \
  build-essential ninja-build git pkg-config \
  gcc-14 g++-14 \
  clang-18 llvm-18 clang-tidy-18 clang-format-18 \
  lcov gcovr cppcheck libgtest-dev \
  gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
  python3-venv python3-pip libusb-1.0-0-dev

echo
echo "Installed versions:"
cmake --version | head -1
gcc-14 --version | head -1
clang-18 --version | head -1
cppcheck --version
arm-none-eabi-gcc --version | head -1
echo "Setup complete."
