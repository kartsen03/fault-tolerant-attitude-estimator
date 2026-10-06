/**
 * @file estimator.h
 * @brief Attitude estimator: timestamped IMU samples in, roll/pitch plus
 *        validity, mode and health out.
 *
 * Wraps the complementary filter with the logic a real loop needs:
 * turning 32-bit microsecond timestamps into validated time steps,
 * handling missing samples, and reporting whether the estimate can be
 * trusted. Fault detection and degraded modes build on this interface.
 */
#ifndef ATT_ESTIMATOR_H
#define ATT_ESTIMATOR_H

#include "att/att_types.h"
#include "att/comp_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Operating mode. */
typedef enum {
    ATT_MODE_INIT = 0,       /**< no valid sample yet */
    ATT_MODE_NOMINAL = 1,    /**< gyro + accelerometer */
    ATT_MODE_GYRO_ONLY = 2,  /**< accelerometer unusable: time-limited gyro coast */
    ATT_MODE_ACCEL_ONLY = 3, /**< gyroscope unusable: accelerometer tilt only */
    ATT_MODE_HOLD = 4,       /**< no usable data: last estimate held */
    ATT_MODE_FAILED = 5      /**< estimate invalid */
} att_mode_t;

/** Overall health of the estimate. */
typedef enum {
    ATT_HEALTH_OK = 0,
    ATT_HEALTH_DEGRADED = 1,
    ATT_HEALTH_FAILED = 2
} att_health_t;

/* Fault flags reported in att_est_output_t.faults. */
#define ATT_FAULT_DROPOUT (1u << 0)   /**< sample missing (driver read failed) */
#define ATT_FAULT_TIMESTAMP (1u << 1) /**< time step rejected */

/** One IMU sample. valid is false when the driver could not read the sensor. */
typedef struct {
    uint32_t t_us;         /**< monotonic timestamp, wraps modulo 2^32 */
    att_vec3_t accel_mps2; /**< specific force */
    att_vec3_t gyro_rps;   /**< angular rate */
    bool valid;
} att_imu_sample_t;

/** Estimator output for one sample. */
typedef struct {
    att_euler_t attitude; /**< roll in (-pi, pi], pitch in [-pi/2, pi/2] */
    bool valid;           /**< false: do not use attitude */
    att_mode_t mode;
    att_health_t health;
    uint32_t faults;      /**< ATT_FAULT_* bits active this sample */
} att_est_output_t;

typedef struct {
    float tau_s;                 /**< complementary filter time constant [s] */
    uint32_t max_dt_us;          /**< largest accepted time step [us] */
    uint32_t dropout_fail_count; /**< consecutive missing samples before FAILED */
} att_est_config_t;

typedef struct {
    uint32_t magic;
    att_est_config_t cfg;
    att_cf_t cf;
    uint32_t t_prev_us;
    bool have_time;
    uint32_t consecutive_dropouts;
    att_vec3_t last_gyro; /**< last good rate, used to extrapolate across dropouts */
    att_est_output_t last;
} att_est_t;

/** Defaults: tau 0.5 s, max step 50 ms, FAILED after 10 missing samples. */
att_status_t att_est_default_config(att_est_config_t *cfg);

att_status_t att_est_init(att_est_t *est, const att_est_config_t *cfg);

/**
 * Process one sample.
 *
 * Time steps come from the 32-bit timestamps by unsigned subtraction, which
 * is correct across counter wraparound. A step of zero or more than
 * max_dt_us (including a clock running backwards) is rejected: the attitude
 * is not updated, and the time reference resynchronises to the rejected
 * sample so that a permanent clock step costs only one sample.
 *
 * A sample with valid == false (driver read failed) is a dropout: the
 * estimate is extrapolated with the last good rate (mode HOLD, DEGRADED)
 * until dropout_fail_count consecutive dropouts, then marked invalid (mode
 * FAILED). The next good sample re-seeds the filter from the accelerometer.
 *
 * @return ATT_OK, ATT_ERR_NULL, ATT_ERR_STATE, ATT_ERR_RANGE (non-finite or
 *         out-of-bounds sample data; state unchanged) or ATT_ERR_TIMESTAMP.
 *         *out is written for every return except ATT_ERR_NULL/ATT_ERR_STATE.
 */
att_status_t att_est_update(att_est_t *est, const att_imu_sample_t *sample, att_est_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ATT_ESTIMATOR_H */
