// Scripted mock of the HAL for driver unit tests. Each expected bus
// transaction is queued in order; any call that differs (operation,
// address, bytes written, read length) is recorded as an error and
// answered with HAL_ERR_BUS.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hal/hal.h"

struct hal_i2c_bus {
    int id;
};

namespace hal_mock {

/** Clear expectations, errors, clock and counters. */
void Reset();

/** Expect hal_i2c_write(addr, data) and answer with result. */
void ExpectWrite(uint8_t addr, std::vector<uint8_t> data, hal_status_t result = HAL_OK);

/** Expect hal_i2c_write_read(addr, wdata, rlen = response.size()). */
void ExpectWriteRead(uint8_t addr, std::vector<uint8_t> wdata, std::vector<uint8_t> response,
                     hal_status_t result = HAL_OK);

/** Mismatches and unexpected calls seen so far. */
const std::vector<std::string> &Errors();

/** Expectations not yet consumed. */
std::size_t Remaining();

/** Timeout argument of the most recent bus call. */
uint32_t LastTimeoutUs();

/** Sum of all hal_delay_us() arguments since Reset(). */
uint64_t TotalDelayUs();

/** Number of bus calls since Reset(). */
std::size_t BusCalls();

}  // namespace hal_mock
