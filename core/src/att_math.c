#include "att_math.h"

#include <math.h>
#include <stddef.h>

float att_wrap_pi(float angle_rad)
{
    /* fmodf is exact and bounded in time for any finite input, unlike a
     * "while (a > pi) a -= 2*pi" loop whose iteration count depends on a. */
    float wrapped = fmodf(angle_rad + ATT_PI_F, ATT_TWO_PI_F); /* (-2pi, 2pi) */
    if (wrapped <= 0.0f) {
        wrapped += ATT_TWO_PI_F; /* (0, 2pi] */
    }
    return wrapped - ATT_PI_F; /* (-pi, pi] */
}

float att_clampf(float value, float lo, float hi)
{
    float out = value;
    if (out < lo) {
        out = lo;
    } else if (out > hi) {
        out = hi;
    } else {
        /* already within [lo, hi] */
    }
    return out;
}

bool att_vec3_within(const att_vec3_t *v, float abs_max)
{
    bool ok = false;
    if (v != NULL) {
        /* fabsf(NaN) <= x is false, so NaN fails; +-Inf exceeds any finite abs_max. */
        ok = (fabsf(v->x) <= abs_max) && (fabsf(v->y) <= abs_max) && (fabsf(v->z) <= abs_max);
    }
    return ok;
}

float att_vec3_norm(const att_vec3_t *v)
{
    return sqrtf((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}

bool att_is_positive_finite(float value)
{
    return (isfinite(value) != 0) && (value > 0.0f);
}
