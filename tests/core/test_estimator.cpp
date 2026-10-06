// Estimator wrapper: timestamps, dropouts, validation.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "att/estimator.h"
#include "imu_kinematics.hpp"

using ftae_test::accel_at_rest;
using ftae_test::deg;

namespace {

constexpr uint32_t kPeriodUs = 10000;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

class EstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        ASSERT_EQ(att_est_default_config(&cfg_), ATT_OK);
        ASSERT_EQ(att_est_init(&est_, &cfg_), ATT_OK);
    }

    att_imu_sample_t Sample(uint32_t t_us, double roll = 0.0, double pitch = 0.0,
                            att_vec3_t gyro = {0.0f, 0.0f, 0.0f}, bool valid = true) const
    {
        return att_imu_sample_t{t_us, accel_at_rest(roll, pitch), gyro, valid};
    }

    att_status_t Feed(const att_imu_sample_t &s) { return att_est_update(&est_, &s, &out_); }

    att_est_config_t cfg_{};
    att_est_t est_{};
    att_est_output_t out_{};
};

}  // namespace

TEST_F(EstimatorTest, Req003FirstSampleSeedsAndReportsNominal)
{
    EXPECT_EQ(est_.last.mode, ATT_MODE_INIT);
    EXPECT_FALSE(est_.last.valid);
    ASSERT_EQ(Feed(Sample(1000, deg(10.0), deg(-5.0))), ATT_OK);
    EXPECT_TRUE(out_.valid);
    EXPECT_EQ(out_.mode, ATT_MODE_NOMINAL);
    EXPECT_EQ(out_.health, ATT_HEALTH_OK);
    EXPECT_EQ(out_.faults, 0u);
    EXPECT_NEAR(out_.attitude.roll_rad, deg(10.0), deg(0.001));
    EXPECT_NEAR(out_.attitude.pitch_rad, deg(-5.0), deg(0.001));
}

TEST_F(EstimatorTest, Req008TimeStepsAreCorrectAcrossCounterWraparound)
{
    // Rotate at 20 deg/s about x while the 32-bit microsecond counter wraps.
    const att_vec3_t w{static_cast<float>(deg(20.0)), 0.0f, 0.0f};
    uint32_t t = 0xFFFFFFFFu - 5u * kPeriodUs;
    double roll = 0.0;
    ASSERT_EQ(Feed(Sample(t, roll, 0.0, w)), ATT_OK);
    for (int i = 0; i < 20; ++i) {
        t += kPeriodUs;  // wraps after five steps
        roll += deg(20.0) * 0.01;
        ASSERT_EQ(Feed(Sample(t, roll, 0.0, w)), ATT_OK) << "step " << i;
        ASSERT_EQ(out_.faults, 0u);
    }
    EXPECT_LT(t, 0x00100000u);  // the counter really wrapped
    EXPECT_NEAR(out_.attitude.roll_rad, roll, deg(0.01));
}

TEST_F(EstimatorTest, Req008RejectsZeroBackwardsAndOversizedSteps)
{
    ASSERT_EQ(Feed(Sample(100000, deg(5.0))), ATT_OK);
    const att_euler_t held = out_.attitude;

    EXPECT_EQ(Feed(Sample(100000, deg(30.0))), ATT_ERR_TIMESTAMP);  // zero step
    EXPECT_EQ(out_.faults, ATT_FAULT_TIMESTAMP);
    EXPECT_FLOAT_EQ(out_.attitude.roll_rad, held.roll_rad);

    EXPECT_EQ(Feed(Sample(90000, deg(30.0))), ATT_ERR_TIMESTAMP);  // backwards
    EXPECT_FLOAT_EQ(out_.attitude.roll_rad, held.roll_rad);

    // Resynchronised to t = 90000, so 90000 + 50001 is one step too large ...
    EXPECT_EQ(Feed(Sample(90000 + 50001, deg(30.0))), ATT_ERR_TIMESTAMP);
    // ... and 50000 later (the configured maximum) is accepted again.
    EXPECT_EQ(Feed(Sample(90000 + 50001 + 50000, deg(5.0))), ATT_OK);
    EXPECT_EQ(out_.faults, 0u);
}

TEST_F(EstimatorTest, Req008PermanentClockStepCostsOneSample)
{
    ASSERT_EQ(Feed(Sample(1000)), ATT_OK);
    EXPECT_EQ(Feed(Sample(5000000)), ATT_ERR_TIMESTAMP);  // clock jumped 5 s
    EXPECT_EQ(Feed(Sample(5000000 + kPeriodUs)), ATT_OK);
    EXPECT_EQ(Feed(Sample(5000000 + 2 * kPeriodUs)), ATT_OK);
}

TEST_F(EstimatorTest, Req010DropoutsExtrapolateThenFailAfterTenSamples)
{
    const att_vec3_t w{static_cast<float>(deg(10.0)), 0.0f, 0.0f};
    uint32_t t = 0;
    ASSERT_EQ(Feed(Sample(t, 0.0, 0.0, w)), ATT_OK);
    for (int i = 1; i <= 9; ++i) {  // nine missing samples: still usable
        t += kPeriodUs;
        ASSERT_EQ(Feed(Sample(t, 0.0, 0.0, {0, 0, 0}, false)), ATT_OK);
        EXPECT_EQ(out_.mode, ATT_MODE_HOLD) << i;
        EXPECT_EQ(out_.health, ATT_HEALTH_DEGRADED) << i;
        EXPECT_TRUE(out_.valid) << i;
        EXPECT_EQ(out_.faults, ATT_FAULT_DROPOUT) << i;
    }
    // Extrapolated with the last good rate: 9 samples x 0.01 s x 10 deg/s.
    EXPECT_NEAR(out_.attitude.roll_rad, deg(0.9), deg(0.01));

    t += kPeriodUs;  // tenth consecutive dropout
    ASSERT_EQ(Feed(Sample(t, 0.0, 0.0, {0, 0, 0}, false)), ATT_OK);
    EXPECT_EQ(out_.mode, ATT_MODE_FAILED);
    EXPECT_EQ(out_.health, ATT_HEALTH_FAILED);
    EXPECT_FALSE(out_.valid);
}

TEST_F(EstimatorTest, Req014RecoveryAfterFailureReseedsFromAccelerometer)
{
    uint32_t t = 0;
    ASSERT_EQ(Feed(Sample(t)), ATT_OK);
    for (int i = 0; i < 10; ++i) {
        t += kPeriodUs;
        ASSERT_EQ(Feed(Sample(t, 0.0, 0.0, {0, 0, 0}, false)), ATT_OK);
    }
    ASSERT_EQ(out_.mode, ATT_MODE_FAILED);
    t += kPeriodUs;
    ASSERT_EQ(Feed(Sample(t, deg(25.0), deg(-15.0))), ATT_OK);  // sensor back, board now tilted
    EXPECT_EQ(out_.mode, ATT_MODE_NOMINAL);
    EXPECT_TRUE(out_.valid);
    EXPECT_NEAR(out_.attitude.roll_rad, deg(25.0), deg(0.001));  // re-seeded, not blended
    EXPECT_NEAR(out_.attitude.pitch_rad, deg(-15.0), deg(0.001));
}

TEST_F(EstimatorTest, Req007RejectsNullUninitialisedAndNonFiniteInput)
{
    att_imu_sample_t s = Sample(1000);
    EXPECT_EQ(att_est_update(nullptr, &s, &out_), ATT_ERR_NULL);
    EXPECT_EQ(att_est_update(&est_, nullptr, &out_), ATT_ERR_NULL);
    EXPECT_EQ(att_est_update(&est_, &s, nullptr), ATT_ERR_NULL);
    EXPECT_EQ(att_est_init(nullptr, &cfg_), ATT_ERR_NULL);
    EXPECT_EQ(att_est_init(&est_, nullptr), ATT_ERR_NULL);
    EXPECT_EQ(att_est_default_config(nullptr), ATT_ERR_NULL);

    att_est_t raw{};
    EXPECT_EQ(att_est_update(&raw, &s, &out_), ATT_ERR_STATE);

    ASSERT_EQ(Feed(s), ATT_OK);
    att_est_t before{};
    std::memcpy(&before, &est_, sizeof est_);
    att_imu_sample_t bad = Sample(2000);
    bad.gyro_rps.y = kNaN;
    EXPECT_EQ(Feed(bad), ATT_ERR_RANGE);
    bad = Sample(2000);
    bad.accel_mps2.z = 5000.0f;  // beyond the input-validation bound
    EXPECT_EQ(Feed(bad), ATT_ERR_RANGE);
    EXPECT_EQ(std::memcmp(&before, &est_, sizeof est_), 0);
}

TEST_F(EstimatorTest, Req007InitRejectsInvalidConfiguration)
{
    att_est_config_t bad = cfg_;
    bad.tau_s = 0.0f;
    EXPECT_EQ(att_est_init(&est_, &bad), ATT_ERR_RANGE);
    bad = cfg_;
    bad.max_dt_us = 0;
    EXPECT_EQ(att_est_init(&est_, &bad), ATT_ERR_RANGE);
    bad = cfg_;
    bad.max_dt_us = 0x80000000u;
    EXPECT_EQ(att_est_init(&est_, &bad), ATT_ERR_RANGE);
    bad = cfg_;
    bad.dropout_fail_count = 0;
    EXPECT_EQ(att_est_init(&est_, &bad), ATT_ERR_RANGE);
}
