// Reference kinematics for tests, in double precision. Independent of the
// code under test so the tests do not share its formulas or its bugs.
#pragma once

#include <cmath>

#include "att/att_types.h"

namespace ftae_test {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

/** Degrees to radians. */
constexpr double deg(double d) { return d * kPi / 180.0; }

/** Specific force measured at rest for a given roll and pitch [m/s^2]. */
inline att_vec3_t accel_at_rest(double roll, double pitch)
{
    return att_vec3_t{static_cast<float>(-kG * std::sin(pitch)),
                      static_cast<float>(kG * std::sin(roll) * std::cos(pitch)),
                      static_cast<float>(kG * std::cos(roll) * std::cos(pitch))};
}

/** Body rates that produce the given Z-Y-X Euler-angle rates [rad/s]. */
inline att_vec3_t body_rates(double roll, double pitch, double roll_dot, double pitch_dot,
                             double yaw_dot)
{
    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    return att_vec3_t{static_cast<float>(roll_dot - sp * yaw_dot),
                      static_cast<float>(cr * pitch_dot + sr * cp * yaw_dot),
                      static_cast<float>(-sr * pitch_dot + cr * cp * yaw_dot)};
}

/** Wrap to (-pi, pi]. */
inline double wrap_pi(double a)
{
    double w = std::fmod(a + kPi, 2.0 * kPi);
    if (w <= 0.0) {
        w += 2.0 * kPi;
    }
    return w - kPi;
}

/** Signed angular difference a - b, taking the short way round. */
inline double angle_diff(double a, double b) { return wrap_pi(a - b); }

}  // namespace ftae_test
