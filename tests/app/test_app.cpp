// Application cycle on the host simulator HAL: driver against the register
// model, absolute-schedule releases, overrun counting, telemetry format.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "app.h"
#include "hal_host.h"

namespace {

constexpr float kPiF = 3.14159265358979f;

class AppTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        hal_host_reset(0);
        mpu6050_model_init(&model_, MPU6050_ADDR_AD0_LOW);
        bus_.imu = &model_;
        ASSERT_EQ(app_default_config(&cfg_), ATT_OK);
    }

    mpu6050_model_t model_{};
    hal_i2c_bus_t bus_{nullptr};
    app_config_t cfg_{};
    app_t app_{};
    app_output_t out_{};
};

std::string Format(const app_output_t &o)
{
    char buf[APP_TELEMETRY_MAX_LEN];
    const size_t n = app_format_telemetry(&o, buf, sizeof buf);
    return std::string(buf, n);
}

// Independent checksum: XOR of every character between '$' and '*'.
std::string ExpectedChecksum(const std::string &body)
{
    unsigned cs = 0;
    for (char c : body) {
        cs ^= static_cast<unsigned char>(c);
    }
    char hex[3];
    std::snprintf(hex, sizeof hex, "%02X", cs);
    return hex;
}

}  // namespace

TEST_F(AppTest, Req009DriverConfiguresTheRegisterModel)
{
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    EXPECT_EQ(app_.imu_status, MPU6050_OK);
    EXPECT_EQ(model_.regs[0x19], 9);     // SMPLRT_DIV -> 100 Hz
    EXPECT_EQ(model_.regs[0x1A], 3);     // DLPF 44 Hz
    EXPECT_EQ(model_.regs[0x1B], 0x08);  // +-500 deg/s
    EXPECT_EQ(model_.regs[0x1C], 0x08);  // +-4 g
    EXPECT_EQ(model_.regs[0x6B], 0x01);  // awake, PLL clock
}

TEST_F(AppTest, Req009EndToEndReadingMatchesTheTiltPresentedToTheSensor)
{
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    const float roll = 20.0f * kPiF / 180.0f;
    const att_vec3_t accel{0.0f, 9.80665f * std::sin(roll), 9.80665f * std::cos(roll)};
    const att_vec3_t gyro{0.0f, 0.0f, 0.0f};
    mpu6050_model_set_inputs(&model_, &accel, &gyro, 25.0f);
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
    }
    EXPECT_EQ(out_.read_status, MPU6050_OK);
    EXPECT_TRUE(out_.est.valid);
    // Quantisation at +-4 g is 1/8192 g, i.e. well under 0.01 deg of tilt.
    EXPECT_NEAR(out_.est.attitude.roll_rad, roll, 0.01f * kPiF / 180.0f);
}

TEST_F(AppTest, Req001ReleasesFollowAnAbsoluteScheduleDespiteOverruns)
{
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    const uint32_t t0 = hal_time_us();  // schedule starts here
    const int n = 1000;
    std::set<int> late;
    for (int k = 0; k < n; ++k) {
        ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
        const uint32_t on_grid = t0 + static_cast<uint32_t>(k + 1) * APP_PERIOD_US;
        if (out_.t_us != on_grid) {
            late.insert(k);
        }
        // Simulated execution time: varies every cycle, one 25 ms overrun at k = 500.
        const uint32_t exec_us = (k == 500) ? 25000u : static_cast<uint32_t>((k * 37) % 9000);
        hal_host_advance_us(exec_us);
    }
    // The 25 ms cycle makes the next two releases late; afterwards every release
    // is back on t0 + k*period, so nothing accumulated.
    EXPECT_EQ(late, (std::set<int>{501, 502}));
    EXPECT_EQ(out_.overruns, 2u);
}

TEST_F(AppTest, Req002OverrunCountAppearsInTelemetry)
{
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    for (int k = 0; k < 20; ++k) {
        ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
        hal_host_advance_us((k == 5 || k == 12) ? 12000u : 1000u);  // two overruns
    }
    EXPECT_EQ(out_.overruns, 2u);
    const std::string line = Format(out_);
    // Field 10 (1-based, after "$ATT") is the overrun count.
    std::vector<std::string> fields;
    std::string cur;
    for (char c : line.substr(1, line.find('*') - 1)) {
        if (c == ',') {
            fields.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    fields.push_back(cur);
    ASSERT_EQ(fields.size(), 11u) << line;
    EXPECT_EQ(fields[9], "2") << line;
}

TEST_F(AppTest, Req002TelemetryLineMatchesTheSpecifiedFormat)
{
    app_output_t o{};
    o.seq = 42;
    o.t_us = 123456789;
    o.est.attitude.roll_rad = -5.12f * kPiF / 180.0f;
    o.est.attitude.pitch_rad = 10.43f * kPiF / 180.0f;
    o.est.valid = true;
    o.est.mode = ATT_MODE_NOMINAL;
    o.est.health = ATT_HEALTH_OK;
    o.est.faults = 0x0001;
    o.overruns = 3;
    o.read_errors = 7;
    const std::string body = "ATT,42,123456,-5.12,10.43,1,NOM,OK,0001,3,7";
    EXPECT_EQ(Format(o), "$" + body + "*" + ExpectedChecksum(body) + "\r\n");
}

TEST_F(AppTest, Req002TelemetryHandlesExtremesAndBadArguments)
{
    app_output_t o{};
    o.est.attitude.roll_rad = kPiF;
    o.est.attitude.pitch_rad = -kPiF / 2.0f;
    o.est.mode = ATT_MODE_FAILED;
    o.est.health = ATT_HEALTH_FAILED;
    o.overruns = 4294967295u;
    std::string line = Format(o);
    EXPECT_NE(line.find(",180.00,-90.00,0,FAIL,FAIL,0000,4294967295,0*"), std::string::npos) << line;

    o.est.attitude.roll_rad = std::nanf("");
    line = Format(o);
    EXPECT_NE(line.find(",0.00,"), std::string::npos) << line;  // NaN never reaches the wire

    char small[10];
    EXPECT_EQ(app_format_telemetry(&o, small, sizeof small), 0u);
    EXPECT_EQ(app_format_telemetry(nullptr, small, sizeof small), 0u);
    EXPECT_EQ(app_format_telemetry(&o, nullptr, 10), 0u);
    EXPECT_STREQ(app_mode_name(static_cast<att_mode_t>(99)), "?");
    EXPECT_STREQ(app_health_name(static_cast<att_health_t>(99)), "?");
}

TEST_F(AppTest, Req010MissingSensorIsReportedEveryCycle)
{
    bus_.imu = nullptr;  // nothing on the bus: every transaction is NACKed
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    EXPECT_EQ(app_.imu_status, MPU6050_ERR_I2C_NACK);
    for (int k = 0; k < 5; ++k) {
        ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
    }
    EXPECT_EQ(out_.read_status, MPU6050_ERR_I2C_NACK);
    EXPECT_EQ(out_.read_errors, 5u);
    EXPECT_FALSE(out_.est.valid);
    EXPECT_EQ(out_.est.faults, ATT_FAULT_DROPOUT);
}

TEST_F(AppTest, Req010BusFaultsBecomeDropoutsAndRecover)
{
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    for (int k = 0; k < 10; ++k) {
        ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
    }
    const mpu6050_model_fault_t faults[] = {MPU6050_MODEL_FAULT_TIMEOUT, MPU6050_MODEL_FAULT_NACK,
                                            MPU6050_MODEL_FAULT_BUS_HIGH};
    const mpu6050_status_t expected[] = {MPU6050_ERR_I2C_TIMEOUT, MPU6050_ERR_I2C_NACK,
                                         MPU6050_ERR_BAD_DATA};
    for (int i = 0; i < 3; ++i) {
        mpu6050_model_set_fault(&model_, faults[i]);
        ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
        EXPECT_EQ(out_.read_status, expected[i]);
        EXPECT_EQ(out_.est.mode, ATT_MODE_HOLD);
        EXPECT_EQ(out_.est.faults, ATT_FAULT_DROPOUT);
    }
    mpu6050_model_set_fault(&model_, MPU6050_MODEL_FAULT_NONE);
    ASSERT_EQ(app_run_cycle(&app_, &out_), ATT_OK);
    EXPECT_EQ(out_.read_status, MPU6050_OK);
    EXPECT_EQ(out_.est.mode, ATT_MODE_NOMINAL);
    EXPECT_EQ(out_.read_errors, 3u);
}

TEST_F(AppTest, Req007AppRejectsNullAndUseBeforeInit)
{
    EXPECT_EQ(app_default_config(nullptr), ATT_ERR_NULL);
    EXPECT_EQ(app_init(nullptr, &bus_, &cfg_), ATT_ERR_NULL);
    EXPECT_EQ(app_init(&app_, nullptr, &cfg_), ATT_ERR_NULL);
    EXPECT_EQ(app_init(&app_, &bus_, nullptr), ATT_ERR_NULL);
    app_config_t bad = cfg_;
    bad.period_us = 0;
    EXPECT_EQ(app_init(&app_, &bus_, &bad), ATT_ERR_RANGE);
    app_t raw{};
    EXPECT_EQ(app_run_cycle(&raw, &out_), ATT_ERR_STATE);
    EXPECT_EQ(app_run_cycle(nullptr, &out_), ATT_ERR_NULL);
    ASSERT_EQ(app_init(&app_, &bus_, &cfg_), ATT_OK);
    EXPECT_EQ(app_run_cycle(&app_, nullptr), ATT_ERR_NULL);
}
