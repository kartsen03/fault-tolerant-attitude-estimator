/**
 * @file hal_host.h
 * @brief Host simulator HAL: simulated microsecond clock, one I2C bus with an
 *        MPU6050 model, and console capture.
 *
 * Time only moves when the code under test waits (hal_delay_us,
 * hal_periodic_wait) or when the harness calls hal_host_advance_us() to
 * stand in for execution time, so runs are deterministic and as fast as
 * the host allows.
 */
#ifndef HAL_HOST_H
#define HAL_HOST_H

#include <stddef.h>
#include <stdint.h>

#include "hal/hal.h"
#include "mpu6050_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Host definition of the opaque bus handle. */
struct hal_i2c_bus {
    mpu6050_model_t *imu; /**< device on the bus, or NULL for an empty bus */
};

/** Reset the simulated clock to start_us and detach any console sink. */
void hal_host_reset(uint32_t start_us);

/** Advance simulated time, e.g. to model the execution time of a cycle. */
void hal_host_advance_us(uint32_t us);

/** Console sink; NULL discards output. */
typedef void (*hal_host_console_fn)(const char *data, size_t len, void *ctx);
void hal_host_set_console(hal_host_console_fn fn, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HAL_HOST_H */
