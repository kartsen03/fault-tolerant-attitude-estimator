#include "att/comp_filter.h"

#include <math.h>
#include <stddef.h>

#include "att_math.h"

#define ATT_CF_MAGIC (0x43463031u) /* "CF01" */

/* Floor for |cos(pitch)| in the Euler-rate transform: cos(85 deg). Past it
 * tan(pitch) grows without bound and roll is ill-conditioned. */
#define ATT_CF_COS_PITCH_MIN (0.0871557f)

static bool cf_is_initialised(const att_cf_t *cf)
{
    return cf->magic == ATT_CF_MAGIC;
}

static bool dt_is_valid(const att_cf_t *cf, float dt_s)
{
    return att_is_positive_finite(dt_s) && (dt_s <= cf->cfg.max_dt_s);
}

/* Integrate body rates over dt with the Z-Y-X Euler-angle kinematics:
 *   roll_rate  = wx + (wy sin(roll) + wz cos(roll)) tan(pitch)
 *   pitch_rate = wy cos(roll) - wz sin(roll)                                */
static att_euler_t cf_predict(const att_euler_t *prev, const att_vec3_t *w, float dt_s)
{
    const float sr = sinf(prev->roll_rad);
    const float cr = cosf(prev->roll_rad);
    const float sp = sinf(prev->pitch_rad);
    float cp = cosf(prev->pitch_rad);
    att_euler_t next;

    /* Pitch is kept in [-pi/2, pi/2], so cos(pitch) >= 0 up to rounding. */
    if (cp < ATT_CF_COS_PITCH_MIN) {
        cp = ATT_CF_COS_PITCH_MIN;
    }

    const float tp = sp / cp;
    const float roll_rate = w->x + (((w->y * sr) + (w->z * cr)) * tp);
    const float pitch_rate = (w->y * cr) - (w->z * sr);

    next.roll_rad = att_wrap_pi(prev->roll_rad + (roll_rate * dt_s));
    next.pitch_rad = att_clampf(prev->pitch_rad + (pitch_rate * dt_s), -ATT_HALF_PI_F, ATT_HALF_PI_F);
    return next;
}

att_status_t att_tilt_from_accel(const att_vec3_t *accel_mps2, att_euler_t *out)
{
    att_status_t status = ATT_OK;

    if ((accel_mps2 == NULL) || (out == NULL)) {
        status = ATT_ERR_NULL;
    } else if (!att_vec3_within(accel_mps2, ATT_ACCEL_ABS_MAX_MPS2)) {
        status = ATT_ERR_RANGE;
    } else if (att_vec3_norm(accel_mps2) < ATT_MIN_TILT_ACCEL_MPS2) {
        status = ATT_ERR_RANGE; /* near free fall: gravity direction unobservable */
    } else {
        const float yz = sqrtf((accel_mps2->y * accel_mps2->y) + (accel_mps2->z * accel_mps2->z));
        /* atan2f(-0.0f, negative) returns -pi; wrapping maps it to +pi. */
        out->roll_rad = att_wrap_pi(atan2f(accel_mps2->y, accel_mps2->z));
        out->pitch_rad = atan2f(-accel_mps2->x, yz);
    }
    return status;
}

att_status_t att_cf_init(att_cf_t *cf, const att_cf_config_t *cfg)
{
    att_status_t status = ATT_OK;

    if ((cf == NULL) || (cfg == NULL)) {
        status = ATT_ERR_NULL;
    } else if ((!att_is_positive_finite(cfg->tau_s)) || (!att_is_positive_finite(cfg->max_dt_s))) {
        status = ATT_ERR_RANGE;
    } else {
        cf->magic = ATT_CF_MAGIC;
        cf->cfg = *cfg;
        cf->est.roll_rad = 0.0f;
        cf->est.pitch_rad = 0.0f;
        cf->innov.roll_rad = 0.0f;
        cf->innov.pitch_rad = 0.0f;
        cf->seeded = false;
    }
    return status;
}

att_status_t att_cf_seed(att_cf_t *cf, const att_vec3_t *accel_mps2)
{
    att_status_t status = ATT_OK;
    att_euler_t tilt = {0.0f, 0.0f};

    if ((cf == NULL) || (accel_mps2 == NULL)) {
        status = ATT_ERR_NULL;
    } else if (!cf_is_initialised(cf)) {
        status = ATT_ERR_STATE;
    } else {
        status = att_tilt_from_accel(accel_mps2, &tilt);
    }

    if (status == ATT_OK) {
        cf->est = tilt;
        cf->innov.roll_rad = 0.0f;
        cf->innov.pitch_rad = 0.0f;
        cf->seeded = true;
    }
    return status;
}

att_status_t att_cf_update(att_cf_t *cf, const att_vec3_t *gyro_rps,
                           const att_vec3_t *accel_mps2, float dt_s)
{
    att_status_t status = ATT_OK;
    att_euler_t tilt = {0.0f, 0.0f};

    if ((cf == NULL) || (gyro_rps == NULL) || (accel_mps2 == NULL)) {
        status = ATT_ERR_NULL;
    } else if (!cf_is_initialised(cf)) {
        status = ATT_ERR_STATE;
    } else if ((!att_vec3_within(gyro_rps, ATT_GYRO_ABS_MAX_RPS)) || (!dt_is_valid(cf, dt_s))) {
        status = ATT_ERR_RANGE;
    } else {
        status = att_tilt_from_accel(accel_mps2, &tilt);
    }

    if (status == ATT_OK) {
        if (!cf->seeded) {
            cf->est = tilt;
            cf->seeded = true;
        } else {
            const att_euler_t pred = cf_predict(&cf->est, gyro_rps, dt_s);
            const float k = dt_s / (cf->cfg.tau_s + dt_s);
            att_euler_t innov;

            /* Roll is circular: take the short way round +-180 deg. Pitch is
             * confined to [-pi/2, pi/2], so a plain difference is correct. */
            innov.roll_rad = att_wrap_pi(tilt.roll_rad - pred.roll_rad);
            innov.pitch_rad = tilt.pitch_rad - pred.pitch_rad;

            cf->est.roll_rad = att_wrap_pi(pred.roll_rad + (k * innov.roll_rad));
            cf->est.pitch_rad = att_clampf(pred.pitch_rad + (k * innov.pitch_rad), -ATT_HALF_PI_F, ATT_HALF_PI_F);
            cf->innov = innov;
        }
    }
    return status;
}

att_status_t att_cf_propagate(att_cf_t *cf, const att_vec3_t *gyro_rps, float dt_s)
{
    att_status_t status = ATT_OK;

    if ((cf == NULL) || (gyro_rps == NULL)) {
        status = ATT_ERR_NULL;
    } else if ((!cf_is_initialised(cf)) || (!cf->seeded)) {
        status = ATT_ERR_STATE;
    } else if ((!att_vec3_within(gyro_rps, ATT_GYRO_ABS_MAX_RPS)) || (!dt_is_valid(cf, dt_s))) {
        status = ATT_ERR_RANGE;
    } else {
        cf->est = cf_predict(&cf->est, gyro_rps, dt_s);
    }
    return status;
}

att_status_t att_cf_set_tau(att_cf_t *cf, float tau_s)
{
    att_status_t status = ATT_OK;

    if (cf == NULL) {
        status = ATT_ERR_NULL;
    } else if (!cf_is_initialised(cf)) {
        status = ATT_ERR_STATE;
    } else if (!att_is_positive_finite(tau_s)) {
        status = ATT_ERR_RANGE;
    } else {
        cf->cfg.tau_s = tau_s;
    }
    return status;
}

att_status_t att_cf_get(const att_cf_t *cf, att_euler_t *out)
{
    att_status_t status = ATT_OK;

    if ((cf == NULL) || (out == NULL)) {
        status = ATT_ERR_NULL;
    } else if ((!cf_is_initialised(cf)) || (!cf->seeded)) {
        status = ATT_ERR_STATE;
    } else {
        *out = cf->est;
    }
    return status;
}

att_status_t att_cf_get_innovation(const att_cf_t *cf, att_euler_t *out)
{
    att_status_t status = ATT_OK;

    if ((cf == NULL) || (out == NULL)) {
        status = ATT_ERR_NULL;
    } else if ((!cf_is_initialised(cf)) || (!cf->seeded)) {
        status = ATT_ERR_STATE;
    } else {
        *out = cf->innov;
    }
    return status;
}
