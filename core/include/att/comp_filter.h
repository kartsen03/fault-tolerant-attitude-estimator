/**
 * @file comp_filter.h
 * @brief Complementary filter fusing gyroscope and accelerometer into
 *        roll and pitch.
 *
 * The gyroscope is accurate over short intervals but its integral drifts;
 * the accelerometer gives drift-free tilt from gravity but is noisy and is
 * disturbed by linear acceleration. Each update:
 *
 *   predicted = previous + EulerRates(previous, gyro) * dt
 *   innov     = wrap(accel_tilt - predicted)
 *   estimate  = predicted + k * innov,      k = dt / (tau + dt)
 *
 * tau is the crossover time constant. Tilt errors decay as exp(-t/tau);
 * a constant gyro bias b settles to an attitude offset of b * tau instead
 * of growing without bound. A larger tau rejects more accelerometer noise
 * and vibration but leaves a larger bias offset and recovers more slowly.
 *
 * Body rates are converted to Euler-angle rates with the Z-Y-X kinematics,
 * so yawing while tilted does not leak into roll and pitch. The transform
 * is clamped at |pitch| = 85 deg, where roll becomes ill-conditioned
 * (gimbal lock); the supported envelope is |pitch| < 85 deg.
 *
 * All state lives in att_cf_t. No dynamic memory, no recursion, no loops.
 */
#ifndef ATT_COMP_FILTER_H
#define ATT_COMP_FILTER_H

#include "att/att_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Smallest specific-force magnitude that defines a tilt [m/s^2] (~0.1 g). */
#define ATT_MIN_TILT_ACCEL_MPS2 (1.0f)

/** Filter configuration. */
typedef struct {
    float tau_s;    /**< crossover time constant [s]; finite, > 0 */
    float max_dt_s; /**< largest accepted time step [s]; finite, > 0 */
} att_cf_config_t;

/** Filter state. Initialise with att_cf_init() before any other call. */
typedef struct {
    uint32_t magic;      /**< set by att_cf_init(); detects use before init */
    att_cf_config_t cfg; /**< active configuration */
    att_euler_t est;     /**< current estimate [rad] */
    att_euler_t innov;   /**< last innovation: accel tilt minus prediction [rad] */
    bool seeded;         /**< true once initialised from an accelerometer sample */
} att_cf_t;

/**
 * Initialise the filter. The estimate stays unseeded until the first
 * att_cf_update() or att_cf_seed().
 * @return ATT_OK, ATT_ERR_NULL, or ATT_ERR_RANGE for a bad configuration.
 */
att_status_t att_cf_init(att_cf_t *cf, const att_cf_config_t *cfg);

/**
 * Set the estimate directly from the accelerometer tilt (no blending).
 * Used at start-up and to recover after a long outage.
 * @return ATT_OK, ATT_ERR_NULL, ATT_ERR_STATE (not initialised), or
 *         ATT_ERR_RANGE (non-finite, out of bounds, or |a| below
 *         ATT_MIN_TILT_ACCEL_MPS2).
 */
att_status_t att_cf_seed(att_cf_t *cf, const att_vec3_t *accel_mps2);

/**
 * Full update: propagate with the gyroscope, correct with the accelerometer.
 * On the first call after init it seeds from the accelerometer instead.
 * On any error the filter state is left unchanged.
 * @param gyro_rps   body angular rate [rad/s]
 * @param accel_mps2 specific force [m/s^2]
 * @param dt_s       time since the previous sample, 0 < dt_s <= max_dt_s
 */
att_status_t att_cf_update(att_cf_t *cf, const att_vec3_t *gyro_rps,
                           const att_vec3_t *accel_mps2, float dt_s);

/**
 * Gyro-only update (degraded mode when the accelerometer is unusable).
 * Requires a seeded filter. On any error the state is left unchanged.
 */
att_status_t att_cf_propagate(att_cf_t *cf, const att_vec3_t *gyro_rps, float dt_s);

/** Change the crossover time constant (used by degraded modes). */
att_status_t att_cf_set_tau(att_cf_t *cf, float tau_s);

/**
 * Read the current estimate.
 * @return ATT_ERR_STATE if the filter has not been seeded yet.
 */
att_status_t att_cf_get(const att_cf_t *cf, att_euler_t *out);

/** Read the innovation of the last att_cf_update() [rad]. */
att_status_t att_cf_get_innovation(const att_cf_t *cf, att_euler_t *out);

/**
 * Roll and pitch implied by a specific-force vector, assuming the only
 * force is gravity: roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2+az^2)).
 */
att_status_t att_tilt_from_accel(const att_vec3_t *accel_mps2, att_euler_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ATT_COMP_FILTER_H */
