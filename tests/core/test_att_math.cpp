// White-box tests of the internal math helpers.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "att_math.h"
#include "imu_kinematics.hpp"

using ftae_test::kPi;

namespace {
constexpr float kPiF = static_cast<float>(kPi);
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
}  // namespace

TEST(Req006Wraparound, WrapPiMapsIntoHalfOpenInterval)
{
    EXPECT_FLOAT_EQ(att_wrap_pi(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(att_wrap_pi(kPiF), kPiF);   // +pi stays +pi
    EXPECT_FLOAT_EQ(att_wrap_pi(-kPiF), kPiF);  // -pi is excluded, maps to +pi
    EXPECT_NEAR(att_wrap_pi(3.0f * kPiF - 0.1f), kPiF - 0.1f, 1e-5f);
    EXPECT_NEAR(att_wrap_pi(-1.5f * kPiF), 0.5f * kPiF, 1e-5f);
    EXPECT_NEAR(att_wrap_pi(2.0f * kPiF), 0.0f, 1e-5f);
    // Near zero the result is limited by float spacing around pi (~2.4e-7).
    EXPECT_NEAR(att_wrap_pi(-1e-6f), -1e-6f, 3e-7f);
}

TEST(Req006Wraparound, WrapPiIsBoundedForLargeInputs)
{
    // A subtract-2pi loop would iterate ~16000 times here; fmodf is O(1).
    const float big = 100000.5f;
    const float w = att_wrap_pi(big);
    EXPECT_GT(w, -kPiF);
    EXPECT_LE(w, kPiF);
    EXPECT_NEAR(w, ftae_test::wrap_pi(static_cast<double>(big)), 1e-2);
}

TEST(Req006Wraparound, ClampLimitsToInterval)
{
    EXPECT_FLOAT_EQ(att_clampf(2.0f, -1.0f, 1.0f), 1.0f);
    EXPECT_FLOAT_EQ(att_clampf(-2.0f, -1.0f, 1.0f), -1.0f);
    EXPECT_FLOAT_EQ(att_clampf(0.25f, -1.0f, 1.0f), 0.25f);
}

TEST(Req007InputValidation, Vec3WithinRejectsNullNonFiniteAndOutOfBounds)
{
    const att_vec3_t ok{1.0f, -2.0f, 3.0f};
    const att_vec3_t edge{5.0f, -5.0f, 5.0f};
    const att_vec3_t big{0.0f, 5.1f, 0.0f};
    const att_vec3_t nan_v{0.0f, 0.0f, kNaN};
    const att_vec3_t inf_v{-kInf, 0.0f, 0.0f};
    EXPECT_TRUE(att_vec3_within(&ok, 5.0f));
    EXPECT_TRUE(att_vec3_within(&edge, 5.0f));
    EXPECT_FALSE(att_vec3_within(&big, 5.0f));
    EXPECT_FALSE(att_vec3_within(&nan_v, 5.0f));
    EXPECT_FALSE(att_vec3_within(&inf_v, 5.0f));
    EXPECT_FALSE(att_vec3_within(nullptr, 5.0f));
}

TEST(Req007InputValidation, PositiveFiniteRejectsZeroNegativeAndNonFinite)
{
    EXPECT_TRUE(att_is_positive_finite(0.01f));
    EXPECT_FALSE(att_is_positive_finite(0.0f));
    EXPECT_FALSE(att_is_positive_finite(-1.0f));
    EXPECT_FALSE(att_is_positive_finite(kNaN));
    EXPECT_FALSE(att_is_positive_finite(kInf));
}
