/**
 * @file app.h
 * @brief Platform-independent application: one 100 Hz estimation cycle and
 *        the telemetry format. The RP2040 FreeRTOS task and the host SIL
 *        runner both drive exactly this code.
 */
#ifndef APP_H
#define APP_H

#include <stddef.h>
#include <stdint.h>

#include "att/estimator.h"
#include "drv/mpu6050.h"
#include "hal/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PERIOD_US (10000u)       /* 100 Hz */
#define APP_TELEMETRY_MAX_LEN (96u)  /* longest line incl. CR LF and NUL is 86 */

typedef struct {
    mpu6050_config_t imu;
    att_est_config_t est;
    uint32_t period_us;
} app_config_t;

/** Result of one cycle, published to telemetry. */
typedef struct {
    uint32_t seq;                /**< cycle counter */
    uint32_t t_us;               /**< sample timestamp */
    att_est_output_t est;        /**< attitude, validity, mode, health, faults */
    mpu6050_status_t read_status;/**< driver result for this cycle */
    uint32_t overruns;           /**< cycles that finished after the next release */
    uint32_t read_errors;        /**< failed sensor reads since start */
} app_output_t;

/** Application state: one instance, statically allocated by the platform. */
typedef struct {
    uint32_t magic;
    app_config_t cfg;
    mpu6050_t imu;
    att_est_t est;
    hal_periodic_t sched;
    mpu6050_status_t imu_status; /**< result of sensor initialisation */
    uint32_t seq;
    uint32_t overruns;
    uint32_t read_errors;
} app_t;

/** Defaults: MPU6050 recommended config, estimator defaults, 10 ms period. */
att_status_t app_default_config(app_config_t *cfg);

/**
 * Initialise the sensor and the estimator and start the periodic schedule.
 * A sensor that fails to initialise is not fatal: every cycle then reports
 * the init error and an invalid estimate. Blocks ~100 ms (sensor reset).
 */
att_status_t app_init(app_t *app, hal_i2c_bus_t *bus, const app_config_t *cfg);

/** Wait for the next release, then read the sensor and update the estimate. */
att_status_t app_run_cycle(app_t *app, app_output_t *out);

/**
 * Format one telemetry line:
 *   $ATT,<seq>,<t_ms>,<roll_deg>,<pitch_deg>,<valid>,<mode>,<health>,
 *        <faults_hex>,<overruns>,<read_errors>*<xor_hex>\r\n
 * Angles have two decimals. The checksum is the XOR of every character
 * between '$' and '*' (NMEA style). Returns the length without the NUL
 * terminator, or 0 if the arguments are invalid or the buffer is too small.
 */
size_t app_format_telemetry(const app_output_t *out, char *buf, size_t len);

/** Short names used in telemetry. */
const char *app_mode_name(att_mode_t mode);
const char *app_health_name(att_health_t health);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
