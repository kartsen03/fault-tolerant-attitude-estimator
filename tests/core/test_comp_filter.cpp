// Unit tests for the complementary filter. Suite names carry the ID of the
// requirement each test verifies (see docs/requirements.md).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "att/comp_filter.h"
#include "imu_kinematics.hpp"

using ftae_test::accel_at_rest;
using ftae_test::angle_diff;
using ftae_test::body_rates;
using ftae_test::deg;

namespace {

constexpr float kDt = 0.01f;  // 100 Hz
constexpr float kTau = 0.5f;
constexpr float kMaxDt = 0.05f;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kPiF = static_cast<float>(ftae_test::kPi);
const att_vec3_t kZeroRate{0.0f, 0.0f, 0.0f};

att_cf_t MakeFilter(float tau = kTau)
{
    att_cf_t cf{};
    const att_cf_config_t cfg{tau, kMaxDt};
    EXPECT_EQ(att_cf_init(&cf, &cfg), ATT_OK);
    return cf;
}

att_euler_t Estimate(const att_cf_t &cf)
{
    att_euler_t e{};
    EXPECT_EQ(att_cf_get(&cf, &e), ATT_OK);
    return e;
}

att_vec3_t Vec(double x, double y, double z)
{
    return att_vec3_t{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

}  // namespace

// ---------------------------------------------------------------------------
// Static tilt (supports REQ-003 accuracy at unit level, noise-free)
// ---------------------------------------------------------------------------

TEST(Req003StaticTilt, HoldsAnyStaticAttitudeInTheEnvelope)
{
    for (double roll_deg = -180.0; roll_deg <= 180.0; roll_deg += 30.0) {
        for (double pitch_deg = -80.0; pitch_deg <= 80.0; pitch_deg += 20.0) {
            att_cf_t cf = MakeFilter();
            const att_vec3_t a = accel_at_rest(deg(roll_deg), deg(pitch_deg));
            for (int i = 0; i < 300; ++i) {
                ASSERT_EQ(att_cf_update(&cf, &kZeroRate, &a, kDt), ATT_OK);
            }
            const att_euler_t e = Estimate(cf);
            EXPECT_NEAR(angle_diff(e.roll_rad, deg(roll_deg)), 0.0, deg(0.001))
                << "roll=" << roll_deg << " pitch=" << pitch_deg;
            EXPECT_NEAR(e.pitch_rad, deg(pitch_deg), deg(0.001))
                << "roll=" << roll_deg << " pitch=" << pitch_deg;
        }
    }
}

TEST(Req003StaticTilt, StepErrorDecaysWithTimeConstantTau)
{
    att_cf_t cf = MakeFilter();
    const att_vec3_t level = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &level), ATT_OK);

    const double step = deg(10.0);
    const att_vec3_t tilted = accel_at_rest(step, 0.0);
    const int n = static_cast<int>(std::lround(kTau / kDt));  // one tau = 50 samples
    for (int i = 0; i < n; ++i) {
        ASSERT_EQ(att_cf_update(&cf, &kZeroRate, &tilted, kDt), ATT_OK);
    }
    const double remaining = (step - Estimate(cf).roll_rad) / step;
    const double k = static_cast<double>(kDt) / (static_cast<double>(kTau) + kDt);
    EXPECT_NEAR(remaining, std::pow(1.0 - k, n), 1e-4);  // exact discrete law
    EXPECT_NEAR(remaining, std::exp(-1.0), 0.01);         // ~e^-1 after one tau
}

// ---------------------------------------------------------------------------
// Constant-rate rotation (supports REQ-003)
// ---------------------------------------------------------------------------

TEST(Req003ConstantRate, TracksConstantRollRateWithoutLag)
{
    att_cf_t cf = MakeFilter();
    const double rate = deg(30.0);
    const att_vec3_t a0 = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &a0), ATT_OK);
    const att_vec3_t w = body_rates(0.0, 0.0, rate, 0.0, 0.0);

    double max_err = 0.0;
    for (int i = 1; i <= 200; ++i) {  // 2 s, 0 -> 60 deg
        const double roll = rate * kDt * i;
        const att_vec3_t a = accel_at_rest(roll, 0.0);
        ASSERT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_OK);
        max_err = std::max(max_err, std::fabs(angle_diff(Estimate(cf).roll_rad, roll)));
    }
    EXPECT_LT(max_err, deg(0.01));
}

TEST(Req003ConstantRate, TracksConstantPitchRateWithoutLag)
{
    att_cf_t cf = MakeFilter();
    const double rate = deg(30.0);
    const att_vec3_t a0 = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &a0), ATT_OK);
    const att_vec3_t w = body_rates(0.0, 0.0, 0.0, rate, 0.0);

    double max_err = 0.0;
    for (int i = 1; i <= 200; ++i) {  // 2 s, 0 -> 60 deg
        const double pitch = rate * kDt * i;
        const att_vec3_t a = accel_at_rest(0.0, pitch);
        ASSERT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_OK);
        max_err = std::max(max_err, std::fabs(Estimate(cf).pitch_rad - pitch));
    }
    EXPECT_LT(max_err, deg(0.01));
}

TEST(Req003ConstantRate, YawingWhileTiltedLeavesRollAndPitchUnchanged)
{
    const double roll = deg(20.0);
    const double pitch = deg(15.0);
    const att_vec3_t a = accel_at_rest(roll, pitch);
    const att_vec3_t w = body_rates(roll, pitch, 0.0, 0.0, deg(90.0));

    // While yawing at 90 deg/s, the x and y gyro axes see large rates even
    // though roll and pitch are constant. A filter that integrated wx and wy
    // directly as roll and pitch rates would be pulled ~tau*23 deg off.
    ASSERT_GT(std::fabs(w.x), deg(20.0));
    ASSERT_GT(std::fabs(w.y), deg(20.0));

    att_cf_t cf = MakeFilter();
    ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);
    for (int i = 0; i < 400; ++i) {
        ASSERT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_OK);
    }
    const att_euler_t e = Estimate(cf);
    EXPECT_NEAR(e.roll_rad, roll, deg(0.01));
    EXPECT_NEAR(e.pitch_rad, pitch, deg(0.01));
}

// ---------------------------------------------------------------------------
// Gyro bias: drift is bounded at b*tau instead of growing (REQ-005)
// ---------------------------------------------------------------------------

TEST(Req005GyroBias, ConstantBiasSettlesToBiasTimesTauAndStaysThere)
{
    att_cf_t cf = MakeFilter();
    const att_vec3_t level = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &level), ATT_OK);
    const att_vec3_t bias = Vec(deg(1.0), deg(-1.0), 0.0);

    for (int i = 0; i < 250; ++i) {  // 5 tau
        ASSERT_EQ(att_cf_update(&cf, &bias, &level, kDt), ATT_OK);
    }
    att_euler_t e = Estimate(cf);
    EXPECT_NEAR(e.roll_rad, deg(1.0) * kTau, deg(0.01));
    EXPECT_NEAR(e.pitch_rad, deg(-1.0) * kTau, deg(0.01));

    for (int i = 0; i < 1500; ++i) {  // 15 s more: no growth
        ASSERT_EQ(att_cf_update(&cf, &bias, &level, kDt), ATT_OK);
    }
    e = Estimate(cf);
    EXPECT_NEAR(e.roll_rad, deg(1.0) * kTau, deg(0.01));
    EXPECT_NEAR(e.pitch_rad, deg(-1.0) * kTau, deg(0.01));
}

TEST(Req005GyroBias, BiasOffsetScalesWithTau)
{
    const att_vec3_t level = accel_at_rest(0.0, 0.0);
    const att_vec3_t bias = Vec(deg(2.0), 0.0, 0.0);
    for (const float tau : {0.25f, 0.5f, 1.0f, 2.0f}) {
        att_cf_t cf = MakeFilter(tau);
        ASSERT_EQ(att_cf_seed(&cf, &level), ATT_OK);
        const int n = static_cast<int>(std::lround(8.0f * tau / kDt));  // 8 tau
        for (int i = 0; i < n; ++i) {
            ASSERT_EQ(att_cf_update(&cf, &bias, &level, kDt), ATT_OK);
        }
        EXPECT_NEAR(Estimate(cf).roll_rad, deg(2.0) * tau, deg(0.01)) << "tau=" << tau;
    }
}

TEST(Req005GyroBias, GyroOnlyPropagationDriftsWithoutBound)
{
    // Contrast case: without the accelerometer the same bias integrates
    // into an error that grows linearly with time (b * t).
    att_cf_t cf = MakeFilter();
    const att_vec3_t level = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &level), ATT_OK);
    const att_vec3_t bias = Vec(deg(1.0), 0.0, 0.0);
    for (int i = 0; i < 2000; ++i) {  // 20 s
        ASSERT_EQ(att_cf_propagate(&cf, &bias, kDt), ATT_OK);
    }
    EXPECT_NEAR(Estimate(cf).roll_rad, deg(20.0), deg(0.05));
}

TEST(Req005GyroBias, InnovationMeasuresBiasInSteadyState)
{
    // In steady state the accelerometer must pull back b*(tau+dt) every
    // sample, so innovation / (tau + dt) estimates the residual gyro bias.
    // The drift monitor uses exactly this residual.
    att_cf_t cf = MakeFilter();
    const att_vec3_t level = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &level), ATT_OK);
    const att_vec3_t bias = Vec(deg(1.5), 0.0, 0.0);
    for (int i = 0; i < 500; ++i) {
        ASSERT_EQ(att_cf_update(&cf, &bias, &level, kDt), ATT_OK);
    }
    att_euler_t innov{};
    ASSERT_EQ(att_cf_get_innovation(&cf, &innov), ATT_OK);
    EXPECT_NEAR(-innov.roll_rad / (kTau + kDt), deg(1.5), deg(0.01));
}

TEST(Req005GyroBias, SetTauChangesTheBiasOffset)
{
    att_cf_t cf = MakeFilter(1.0f);
    const att_vec3_t level = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &level), ATT_OK);
    const att_vec3_t bias = Vec(deg(1.0), 0.0, 0.0);
    ASSERT_EQ(att_cf_set_tau(&cf, 0.1f), ATT_OK);
    for (int i = 0; i < 200; ++i) {
        ASSERT_EQ(att_cf_update(&cf, &bias, &level, kDt), ATT_OK);
    }
    EXPECT_NEAR(Estimate(cf).roll_rad, deg(1.0) * 0.1, deg(0.01));
}

// ---------------------------------------------------------------------------
// Angle wraparound and output ranges (REQ-006)
// ---------------------------------------------------------------------------

TEST(Req006Wraparound, RollCrossesPlusMinus180Continuously)
{
    att_cf_t cf = MakeFilter();
    const double roll0 = deg(170.0);
    const double rate = deg(20.0);
    const att_vec3_t a0 = accel_at_rest(roll0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &a0), ATT_OK);
    const att_vec3_t w = body_rates(0.0, 0.0, rate, 0.0, 0.0);

    for (int i = 1; i <= 100; ++i) {  // 1 s: 170 -> 190 (= -170) deg
        const double roll = roll0 + rate * kDt * i;
        const att_vec3_t a = accel_at_rest(roll, 0.0);
        ASSERT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_OK);
        const att_euler_t e = Estimate(cf);
        ASSERT_GT(e.roll_rad, -kPiF);
        ASSERT_LE(e.roll_rad, kPiF);
        ASSERT_LT(std::fabs(angle_diff(e.roll_rad, roll)), deg(0.01)) << "step " << i;
    }
    EXPECT_NEAR(Estimate(cf).roll_rad, deg(-170.0), deg(0.01));
}

TEST(Req006Wraparound, CorrectionTakesTheShortWayAcross180)
{
    // Estimate at +179 deg, accelerometer says -179 deg: the two are 2 deg
    // apart through 180, not 358 deg apart through 0.
    att_cf_t cf = MakeFilter();
    const att_vec3_t from = accel_at_rest(deg(179.0), 0.0);
    const att_vec3_t to = accel_at_rest(deg(-179.0), 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &from), ATT_OK);
    for (int i = 0; i < 300; ++i) {
        ASSERT_EQ(att_cf_update(&cf, &kZeroRate, &to, kDt), ATT_OK);
        ASSERT_GT(std::fabs(Estimate(cf).roll_rad), deg(178.99)) << "step " << i;
    }
    EXPECT_NEAR(angle_diff(Estimate(cf).roll_rad, deg(-179.0)), 0.0, deg(0.01));
}

TEST(Req006Wraparound, UpsideDownWithNegativeZeroReportsPlusPi)
{
    // atan2f(-0.0f, -g) is -pi; the documented range (-pi, pi] needs +pi.
    att_euler_t e{};
    const att_vec3_t a{0.0f, -0.0f, -9.80665f};
    ASSERT_EQ(att_tilt_from_accel(&a, &e), ATT_OK);
    EXPECT_FLOAT_EQ(e.roll_rad, kPiF);
    EXPECT_FLOAT_EQ(e.pitch_rad, 0.0f);
}

TEST(Req006Wraparound, OutputsStayInDocumentedRanges)
{
    for (double roll_deg = -180.0; roll_deg <= 180.0; roll_deg += 15.0) {
        for (double pitch_deg = -90.0; pitch_deg <= 90.0; pitch_deg += 15.0) {
            att_cf_t cf = MakeFilter();
            const att_vec3_t a = accel_at_rest(deg(roll_deg), deg(pitch_deg));
            const att_vec3_t w = Vec(deg(40.0), deg(-25.0), deg(60.0));
            ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);
            for (int i = 0; i < 50; ++i) {
                ASSERT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_OK);
                const att_euler_t e = Estimate(cf);
                ASSERT_GT(e.roll_rad, -kPiF);
                ASSERT_LE(e.roll_rad, kPiF);
                ASSERT_GE(e.pitch_rad, -kPiF / 2.0f);
                ASSERT_LE(e.pitch_rad, kPiF / 2.0f);
            }
        }
    }
}

TEST(Req006Wraparound, NearVerticalPitchStaysFiniteAndClamped)
{
    // Close to pitch = 90 deg tan(pitch) explodes; the transform is clamped.
    att_cf_t cf = MakeFilter();
    const att_vec3_t a = accel_at_rest(0.0, deg(89.0));
    ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);
    const att_vec3_t yaw = Vec(0.0, 0.0, deg(30.0));
    const att_vec3_t up = Vec(0.0, deg(50.0), 0.0);
    for (int i = 0; i < 200; ++i) {
        ASSERT_EQ(att_cf_update(&cf, &yaw, &a, kDt), ATT_OK);
        ASSERT_EQ(att_cf_propagate(&cf, &up, kDt), ATT_OK);  // pushes past 90
        const att_euler_t e = Estimate(cf);
        ASSERT_TRUE(std::isfinite(e.roll_rad));
        ASSERT_TRUE(std::isfinite(e.pitch_rad));
        ASSERT_LE(std::fabs(e.pitch_rad), kPiF / 2.0f);
    }
}

// ---------------------------------------------------------------------------
// Input validation (REQ-007) and time-step validation (REQ-008)
// ---------------------------------------------------------------------------

TEST(Req007InputValidation, InitRejectsNullAndInvalidConfig)
{
    att_cf_t cf{};
    const att_cf_config_t good{kTau, kMaxDt};
    EXPECT_EQ(att_cf_init(nullptr, &good), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_init(&cf, nullptr), ATT_ERR_NULL);
    for (const float bad : {0.0f, -1.0f, kNaN, kInf}) {
        const att_cf_config_t bad_tau{bad, kMaxDt};
        const att_cf_config_t bad_dt{kTau, bad};
        EXPECT_EQ(att_cf_init(&cf, &bad_tau), ATT_ERR_RANGE) << bad;
        EXPECT_EQ(att_cf_init(&cf, &bad_dt), ATT_ERR_RANGE) << bad;
    }
    EXPECT_EQ(att_cf_init(&cf, &good), ATT_OK);
}

TEST(Req007InputValidation, EveryFunctionRejectsNullPointers)
{
    att_cf_t cf = MakeFilter();
    const att_vec3_t a = accel_at_rest(0.0, 0.0);
    att_euler_t e{};
    ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);

    EXPECT_EQ(att_cf_seed(nullptr, &a), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_seed(&cf, nullptr), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_update(nullptr, &kZeroRate, &a, kDt), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_update(&cf, nullptr, &a, kDt), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_update(&cf, &kZeroRate, nullptr, kDt), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_propagate(nullptr, &kZeroRate, kDt), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_propagate(&cf, nullptr, kDt), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_set_tau(nullptr, 1.0f), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_get(nullptr, &e), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_get(&cf, nullptr), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_get_innovation(nullptr, &e), ATT_ERR_NULL);
    EXPECT_EQ(att_cf_get_innovation(&cf, nullptr), ATT_ERR_NULL);
    EXPECT_EQ(att_tilt_from_accel(nullptr, &e), ATT_ERR_NULL);
    EXPECT_EQ(att_tilt_from_accel(&a, nullptr), ATT_ERR_NULL);
}

TEST(Req007InputValidation, RejectsUseBeforeInitAndBeforeSeed)
{
    att_cf_t raw{};  // magic == 0: never initialised
    const att_vec3_t a = accel_at_rest(0.0, 0.0);
    att_euler_t e{};
    EXPECT_EQ(att_cf_seed(&raw, &a), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_update(&raw, &kZeroRate, &a, kDt), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_propagate(&raw, &kZeroRate, kDt), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_set_tau(&raw, 1.0f), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_get(&raw, &e), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_get_innovation(&raw, &e), ATT_ERR_STATE);

    att_cf_t cf = MakeFilter();  // initialised but not seeded
    EXPECT_EQ(att_cf_get(&cf, &e), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_get_innovation(&cf, &e), ATT_ERR_STATE);
    EXPECT_EQ(att_cf_propagate(&cf, &kZeroRate, kDt), ATT_ERR_STATE);
}

TEST(Req007InputValidation, RejectsNonFiniteAndOutOfBoundsInputs)
{
    att_cf_t cf = MakeFilter();
    const att_vec3_t a = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);
    att_euler_t e{};

    const att_vec3_t bad_rates[] = {
        {kNaN, 0.0f, 0.0f}, {0.0f, kInf, 0.0f}, {0.0f, 0.0f, -kInf}, {101.0f, 0.0f, 0.0f}};
    for (const att_vec3_t &w : bad_rates) {
        EXPECT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_ERR_RANGE);
        EXPECT_EQ(att_cf_propagate(&cf, &w, kDt), ATT_ERR_RANGE);
    }

    const att_vec3_t bad_accels[] = {{kNaN, 0.0f, 9.8f},
                                     {0.0f, kInf, 9.8f},
                                     {0.0f, 0.0f, 1001.0f},
                                     {0.1f, 0.1f, 0.1f}};  // ~free fall: no tilt
    for (const att_vec3_t &bad : bad_accels) {
        EXPECT_EQ(att_cf_update(&cf, &kZeroRate, &bad, kDt), ATT_ERR_RANGE);
        EXPECT_EQ(att_cf_seed(&cf, &bad), ATT_ERR_RANGE);
        EXPECT_EQ(att_tilt_from_accel(&bad, &e), ATT_ERR_RANGE);
    }

    for (const float bad_tau : {0.0f, -0.5f, kNaN, kInf}) {
        EXPECT_EQ(att_cf_set_tau(&cf, bad_tau), ATT_ERR_RANGE);
    }
}

TEST(Req007InputValidation, FailedCallsLeaveStateUnchanged)
{
    att_cf_t cf = MakeFilter();
    const att_vec3_t a = accel_at_rest(deg(12.0), deg(-7.0));
    const att_vec3_t w = Vec(deg(5.0), deg(3.0), 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);
    ASSERT_EQ(att_cf_update(&cf, &w, &a, kDt), ATT_OK);

    att_cf_t before{};
    std::memcpy(&before, &cf, sizeof cf);
    const att_vec3_t nan_rate{kNaN, 0.0f, 0.0f};
    const att_vec3_t nan_accel{0.0f, kNaN, 9.8f};
    EXPECT_NE(att_cf_update(&cf, &nan_rate, &a, kDt), ATT_OK);
    EXPECT_NE(att_cf_update(&cf, &w, &nan_accel, kDt), ATT_OK);
    EXPECT_NE(att_cf_update(&cf, &w, &a, 0.0f), ATT_OK);
    EXPECT_NE(att_cf_propagate(&cf, &w, -kDt), ATT_OK);
    EXPECT_NE(att_cf_seed(&cf, &nan_accel), ATT_OK);
    EXPECT_NE(att_cf_set_tau(&cf, kNaN), ATT_OK);
    EXPECT_EQ(std::memcmp(&before, &cf, sizeof cf), 0);
}

TEST(Req008TimeStep, FilterRejectsZeroNegativeNonFiniteAndOversizedSteps)
{
    att_cf_t cf = MakeFilter();
    const att_vec3_t a = accel_at_rest(0.0, 0.0);
    ASSERT_EQ(att_cf_seed(&cf, &a), ATT_OK);
    for (const float dt : {0.0f, -0.01f, kNaN, kInf, kMaxDt * 1.01f}) {
        EXPECT_EQ(att_cf_update(&cf, &kZeroRate, &a, dt), ATT_ERR_RANGE) << dt;
        EXPECT_EQ(att_cf_propagate(&cf, &kZeroRate, dt), ATT_ERR_RANGE) << dt;
    }
    EXPECT_EQ(att_cf_update(&cf, &kZeroRate, &a, kMaxDt), ATT_OK);  // boundary accepted
}
