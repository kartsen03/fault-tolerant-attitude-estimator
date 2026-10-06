# Fault-tolerant attitude estimator

Firmware and host tooling for estimating pitch and roll from an MPU6050
IMU (accelerometer + gyroscope) at a fixed 100 Hz rate, with sensor fault
monitoring and software-in-the-loop (SIL) validation against simulated
ground truth.

**Status: work in progress.** The portable C11 complementary filter and its
unit tests are in place; the driver, firmware, fault monitor, SIL harness
and verification reports follow.

## Build and test (host)

Requires CMake 3.22+, Ninja, GCC 14 or Clang 18, and GoogleTest
(`tools/setup_wsl.sh` installs everything on Ubuntu 24.04 / WSL2).

```bash
cmake --preset gcc
cmake --build --preset gcc
ctest --preset gcc
```

Other presets: `clang`, `sanitize` (ASan + UBSan), `coverage`, `mcdc`.
