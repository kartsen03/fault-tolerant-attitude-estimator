/**
 * @file hal.h
 * @brief Hardware abstraction layer: I2C, monotonic time, periodic
 *        scheduling and console output.
 *
 * The driver and application call only these functions. Each platform
 * links exactly one implementation (link-time substitution):
 *  - hal/rp2040/hal_rp2040.c : Raspberry Pi Pico SDK + FreeRTOS
 *  - hal/host/hal_host.c     : host simulator (simulated clock and a
 *                              register-level MPU6050 model)
 *  - tests/mocks/hal_mock.cpp: scripted mock for driver unit tests
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_OK = 0,
    HAL_ERR_PARAM = 1,   /**< invalid argument */
    HAL_ERR_TIMEOUT = 2, /**< bus transaction did not finish in time */
    HAL_ERR_NACK = 3,    /**< device did not acknowledge its address */
    HAL_ERR_BUS = 4      /**< any other bus failure (short transfer, arbitration) */
} hal_status_t;

/** Opaque I2C bus handle. Each platform defines struct hal_i2c_bus. */
typedef struct hal_i2c_bus hal_i2c_bus_t;

/**
 * Write len bytes to a 7-bit address, then STOP.
 * The whole transfer must finish within timeout_us.
 */
hal_status_t hal_i2c_write(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *data, size_t len,
                           uint32_t timeout_us);

/**
 * Write wlen bytes (normally a register address), repeated START, read
 * rlen bytes, STOP. Each phase must finish within timeout_us.
 */
hal_status_t hal_i2c_write_read(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *wdata,
                                size_t wlen, uint8_t *rdata, size_t rlen, uint32_t timeout_us);

/** Monotonic microsecond counter. Wraps modulo 2^32 (about 71.6 minutes). */
uint32_t hal_time_us(void);

/** Wait at least us microseconds. May block the calling task. */
void hal_delay_us(uint32_t us);

/**
 * Periodic release on an absolute schedule: release k happens at
 * t0 + k * period, independent of how long earlier cycles took
 * (vTaskDelayUntil semantics on the target).
 */
typedef struct {
    uint32_t period_us;    /**< release period */
    uint32_t last_release; /**< platform time base: ticks (target) or us (host) */
    bool started;          /**< set by hal_periodic_init */
} hal_periodic_t;

/** Start a periodic schedule; the first release is one period from now. */
hal_status_t hal_periodic_init(hal_periodic_t *p, uint32_t period_us);

/**
 * Block until the next release. *missed is set true when that release
 * time had already passed on entry, i.e. the previous cycle overran.
 */
hal_status_t hal_periodic_wait(hal_periodic_t *p, bool *missed);

/** Write len bytes to the console (UART on the target, stdout on the host). */
void hal_console_write(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_H */
