#include "att/estimator.h"

#include <stddef.h>

#include "att_math.h"

#define ATT_EST_MAGIC (0x45535430u) /* "EST0" */
#define ATT_US_TO_S (1.0e-6f)
/* Steps of 2^31 us or more mean the clock went backwards (modular arithmetic). */
#define ATT_HALF_RANGE_U32 (0x80000000u)

static bool est_config_is_valid(const att_est_config_t *cfg)
{
    return att_is_positive_finite(cfg->tau_s) && (cfg->max_dt_us > 0u) &&
           (cfg->max_dt_us < ATT_HALF_RANGE_U32) && (cfg->dropout_fail_count > 0u);
}

static void set_output(att_est_t *est, att_mode_t mode, att_health_t health, bool valid, uint32_t faults)
{
    att_euler_t attitude = {0.0f, 0.0f};
    /* Nothing to report before the filter is seeded. */
    const bool seeded = (att_cf_get(&est->cf, &attitude) == ATT_OK);
    est->last.attitude = attitude;
    est->last.valid = valid && seeded;
    est->last.mode = mode;
    est->last.health = health;
    est->last.faults = faults;
}

/* A sample with data present. Seeds the filter on the first sample or after a
 * failure; otherwise runs the full update, falling back to gyro-only when the
 * accelerometer gives no usable tilt (|a| below the free-fall threshold). */
static void process_valid(att_est_t *est, const att_imu_sample_t *s, float dt_s)
{
    const bool reseed = (!est->cf.seeded) || (est->last.mode == ATT_MODE_FAILED);
    att_status_t st;

    if (reseed) {
        st = att_cf_seed(&est->cf, &s->accel_mps2);
    } else {
        st = att_cf_update(&est->cf, &s->gyro_rps, &s->accel_mps2, dt_s);
        if (st != ATT_OK) {
            st = att_cf_propagate(&est->cf, &s->gyro_rps, dt_s);
        }
    }
    est->consecutive_dropouts = 0u;
    est->last_gyro = s->gyro_rps;

    if (st == ATT_OK) {
        set_output(est, ATT_MODE_NOMINAL, ATT_HEALTH_OK, true, 0u);
    } else {
        set_output(est, est->last.mode, ATT_HEALTH_FAILED, false, 0u);
    }
}

/* A missing sample: extrapolate with the last good rate for a bounded number
 * of samples, then declare the estimate failed. */
static void process_dropout(att_est_t *est, float dt_s)
{
    if (est->consecutive_dropouts < UINT32_MAX) {
        est->consecutive_dropouts += 1u;
    }

    if ((est->cf.seeded) && (est->last.mode != ATT_MODE_FAILED) &&
        (est->consecutive_dropouts < est->cfg.dropout_fail_count)) {
        (void)att_cf_propagate(&est->cf, &est->last_gyro, dt_s);
        set_output(est, ATT_MODE_HOLD, ATT_HEALTH_DEGRADED, true, ATT_FAULT_DROPOUT);
    } else if (est->cf.seeded) {
        set_output(est, ATT_MODE_FAILED, ATT_HEALTH_FAILED, false, ATT_FAULT_DROPOUT);
    } else {
        set_output(est, ATT_MODE_INIT, ATT_HEALTH_FAILED, false, ATT_FAULT_DROPOUT);
    }
}

att_status_t att_est_default_config(att_est_config_t *cfg)
{
    att_status_t status = ATT_OK;
    if (cfg == NULL) {
        status = ATT_ERR_NULL;
    } else {
        cfg->tau_s = 0.5f;
        cfg->max_dt_us = 50000u;
        cfg->dropout_fail_count = 10u;
    }
    return status;
}

att_status_t att_est_init(att_est_t *est, const att_est_config_t *cfg)
{
    att_status_t status = ATT_OK;

    if ((est == NULL) || (cfg == NULL)) {
        status = ATT_ERR_NULL;
    } else if (!est_config_is_valid(cfg)) {
        status = ATT_ERR_RANGE;
    } else {
        const att_cf_config_t cf_cfg = {cfg->tau_s, (float)cfg->max_dt_us * ATT_US_TO_S};
        status = att_cf_init(&est->cf, &cf_cfg);
    }

    if (status == ATT_OK) {
        est->cfg = *cfg;
        est->t_prev_us = 0u;
        est->have_time = false;
        est->consecutive_dropouts = 0u;
        est->last_gyro.x = 0.0f;
        est->last_gyro.y = 0.0f;
        est->last_gyro.z = 0.0f;
        est->magic = ATT_EST_MAGIC;
        set_output(est, ATT_MODE_INIT, ATT_HEALTH_FAILED, false, 0u);
    }
    return status;
}

att_status_t att_est_update(att_est_t *est, const att_imu_sample_t *sample, att_est_output_t *out)
{
    att_status_t status = ATT_OK;

    if ((est == NULL) || (sample == NULL) || (out == NULL)) {
        status = ATT_ERR_NULL;
    } else if (est->magic != ATT_EST_MAGIC) {
        status = ATT_ERR_STATE;
    } else if ((sample->valid) &&
               ((!att_vec3_within(&sample->accel_mps2, ATT_ACCEL_ABS_MAX_MPS2)) ||
                (!att_vec3_within(&sample->gyro_rps, ATT_GYRO_ABS_MAX_RPS)))) {
        status = ATT_ERR_RANGE; /* state unchanged, held estimate reported */
        *out = est->last;
    } else {
        /* Unsigned subtraction gives the right step across 2^32 wraparound;
         * a clock that went backwards shows up as a huge step. */
        const uint32_t dt_us = sample->t_us - est->t_prev_us;
        const bool dt_ok = (dt_us > 0u) && (dt_us <= est->cfg.max_dt_us);

        if ((est->have_time) && (!dt_ok)) {
            /* Reject the sample but resynchronise to its time, so a permanent
             * clock step costs one sample instead of every later one. */
            est->t_prev_us = sample->t_us;
            est->last.faults = ATT_FAULT_TIMESTAMP;
            status = ATT_ERR_TIMESTAMP;
        } else {
            const float dt_s = (float)dt_us * ATT_US_TO_S;
            est->t_prev_us = sample->t_us;
            est->have_time = true;
            if (sample->valid) {
                process_valid(est, sample, dt_s);
            } else {
                process_dropout(est, dt_s);
            }
        }
        *out = est->last;
    }
    return status;
}
