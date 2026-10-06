/**
 * @file att_math.h
 * @brief Internal math helpers for the attitude core (not public API).
 *
 * These helpers state their preconditions instead of returning status
 * codes; every public function validates its inputs before calling them.
 */
#ifndef ATT_MATH_H
#define ATT_MATH_H

#include "att/att_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ATT_PI_F (3.14159265358979f)
#define ATT_TWO_PI_F (6.28318530717959f)
#define ATT_HALF_PI_F (1.57079632679490f)
#define ATT_DEG_TO_RAD_F (ATT_PI_F / 180.0f)
#define ATT_RAD_TO_DEG_F (180.0f / ATT_PI_F)

/** Wrap an angle to (-pi, pi]. Precondition: angle_rad is finite. */
float att_wrap_pi(float angle_rad);

/** Clamp value to [lo, hi]. Precondition: lo <= hi, all finite. */
float att_clampf(float value, float lo, float hi);

/** True if v is non-NULL and every component is finite with |c| <= abs_max. */
bool att_vec3_within(const att_vec3_t *v, float abs_max);

/** Euclidean norm. Precondition: v is non-NULL and finite. */
float att_vec3_norm(const att_vec3_t *v);

/** True if value is finite and strictly positive. */
bool att_is_positive_finite(float value);

#ifdef __cplusplus
}
#endif

#endif /* ATT_MATH_H */
