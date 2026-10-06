// MPU6050 driver tests against the scripted mock HAL (REQ-009).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "drv/mpu6050.h"
#include "hal_mock.hpp"

namespace {

constexpr uint8_t kAddr = MPU6050_ADDR_AD0_LOW;
constexpr float kG = 9.80665f;
constexpr float kDegToRad = 0.0174532925199433f;

class Mpu6050Test : public ::testing::Test {
protected:
    void SetUp() override
    {
        hal_mock::Reset();
        ASSERT_EQ(mpu6050_default_config(&cfg_), MPU6050_OK);
    }

    void TearDown() override
    {
        for (const std::string &e : hal_mock::Errors()) {
            ADD_FAILURE() << "mock: " << e;
        }
    }

    // Queue the bus traffic of a successful init for configuration cfg.
    static void ExpectInit(const mpu6050_config_t &cfg)
    {
        const uint8_t a = cfg.address;
        const uint8_t gyro = static_cast<uint8_t>(cfg.gyro_fs << 3);
        const uint8_t accel = static_cast<uint8_t>(cfg.accel_fs << 3);
        const uint8_t dlpf = static_cast<uint8_t>(cfg.dlpf);
        hal_mock::ExpectWriteRead(a, {MPU6050_REG_WHO_AM_I}, {MPU6050_WHO_AM_I_VALUE});
        hal_mock::ExpectWrite(a, {MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_RESET});
        hal_mock::ExpectWrite(a, {MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLK_PLL_X});
        hal_mock::ExpectWrite(a, {MPU6050_REG_SMPLRT_DIV, cfg.sample_rate_div, dlpf, gyro, accel});
        hal_mock::ExpectWriteRead(a, {MPU6050_REG_SMPLRT_DIV}, {cfg.sample_rate_div, dlpf, gyro, accel});
        hal_mock::ExpectWriteRead(a, {MPU6050_REG_PWR_MGMT_1}, {MPU6050_PWR_CLK_PLL_X});
    }

    void InitDevice()
    {
        ExpectInit(cfg_);
        ASSERT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_OK);
    }

    static void ExpectFrame(const std::vector<uint8_t> &frame, hal_status_t result = HAL_OK)
    {
        hal_mock::ExpectWriteRead(kAddr, {MPU6050_REG_ACCEL_XOUT_H}, frame, result);
    }

    hal_i2c_bus_t bus_{0};
    mpu6050_config_t cfg_{};
    mpu6050_t dev_{};
};

// A plausible frame: accel (+1 g, -1 g, max), temp 24 degC, gyro (+10, -10, min) deg/s.
const std::vector<uint8_t> kFrame = {0x20, 0x00, 0xE0, 0x00, 0x7F, 0xFF,  // accel
                                     0xEF, 0x5C,                          // temp raw -4260
                                     0x02, 0x8F, 0xFD, 0x71, 0x80, 0x00}; // gyro 655, -655, -32768

}  // namespace

// --- Configuration -----------------------------------------------------------

TEST_F(Mpu6050Test, Req009DefaultConfigMatchesTheDesign)
{
    EXPECT_EQ(cfg_.address, 0x68);
    EXPECT_EQ(cfg_.accel_fs, MPU6050_ACCEL_FS_4G);
    EXPECT_EQ(cfg_.gyro_fs, MPU6050_GYRO_FS_500DPS);
    EXPECT_EQ(cfg_.dlpf, MPU6050_DLPF_44HZ);
    float rate = 0.0f;
    ASSERT_EQ(mpu6050_sample_rate_hz(&cfg_, &rate), MPU6050_OK);
    EXPECT_FLOAT_EQ(rate, 100.0f);
    EXPECT_EQ(mpu6050_default_config(nullptr), MPU6050_ERR_PARAM);
}

TEST_F(Mpu6050Test, Req009SampleRateFollowsDividerAndDlpf)
{
    float rate = 0.0f;
    cfg_.sample_rate_div = 0;
    ASSERT_EQ(mpu6050_sample_rate_hz(&cfg_, &rate), MPU6050_OK);
    EXPECT_FLOAT_EQ(rate, 1000.0f);
    cfg_.dlpf = MPU6050_DLPF_260HZ;  // gyro output rate becomes 8 kHz
    cfg_.sample_rate_div = 7;
    ASSERT_EQ(mpu6050_sample_rate_hz(&cfg_, &rate), MPU6050_OK);
    EXPECT_FLOAT_EQ(rate, 1000.0f);
    EXPECT_EQ(mpu6050_sample_rate_hz(nullptr, &rate), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_sample_rate_hz(&cfg_, nullptr), MPU6050_ERR_PARAM);
    cfg_.address = 0x50;
    EXPECT_EQ(mpu6050_sample_rate_hz(&cfg_, &rate), MPU6050_ERR_PARAM);
}

// --- Initialisation ----------------------------------------------------------

TEST_F(Mpu6050Test, Req009InitWritesTheDocumentedRegisterSequence)
{
    // WHO_AM_I, reset, 100 ms wait, wake on the gyro PLL, one burst for
    // SMPLRT_DIV=9 / CONFIG=3 (44 Hz) / GYRO_CONFIG=0x08 (500 dps) /
    // ACCEL_CONFIG=0x08 (4 g), then read everything back.
    hal_mock::ExpectWriteRead(kAddr, {0x75}, {0x68});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x80});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x01});
    hal_mock::ExpectWrite(kAddr, {0x19, 0x09, 0x03, 0x08, 0x08});
    hal_mock::ExpectWriteRead(kAddr, {0x19}, {0x09, 0x03, 0x08, 0x08});
    hal_mock::ExpectWriteRead(kAddr, {0x6B}, {0x01});

    EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_OK);
    EXPECT_EQ(hal_mock::Remaining(), 0u);
    EXPECT_GE(hal_mock::TotalDelayUs(), 100000u);
    EXPECT_EQ(hal_mock::LastTimeoutUs(), cfg_.i2c_timeout_us);
}

TEST_F(Mpu6050Test, Req009InitUsesTheAlternateAddress)
{
    cfg_.address = MPU6050_ADDR_AD0_HIGH;  // WHO_AM_I still reads 0x68
    ExpectInit(cfg_);
    EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_OK);
    EXPECT_EQ(hal_mock::Remaining(), 0u);
}

TEST_F(Mpu6050Test, Req009InitRejectsWrongIdentityWithoutFurtherTraffic)
{
    hal_mock::ExpectWriteRead(kAddr, {0x75}, {0x70});  // an MPU6500 answers 0x70
    EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_ERR_WHO_AM_I);
    EXPECT_EQ(hal_mock::BusCalls(), 1u);
    mpu6050_raw_t raw{};
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_STATE);
}

TEST_F(Mpu6050Test, Req009InitReportsTimeoutNackAndBusErrorsDistinctly)
{
    const struct {
        hal_status_t hal;
        mpu6050_status_t expected;
    } cases[] = {{HAL_ERR_TIMEOUT, MPU6050_ERR_I2C_TIMEOUT},
                 {HAL_ERR_NACK, MPU6050_ERR_I2C_NACK},
                 {HAL_ERR_BUS, MPU6050_ERR_I2C_BUS},
                 {HAL_ERR_PARAM, MPU6050_ERR_I2C_BUS}};
    for (const auto &c : cases) {
        hal_mock::Reset();
        hal_mock::ExpectWriteRead(kAddr, {0x75}, {0x00}, c.hal);
        EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), c.expected);
    }
}

TEST_F(Mpu6050Test, Req009InitStopsAtTheFirstFailingStep)
{
    hal_mock::ExpectWriteRead(kAddr, {0x75}, {0x68});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x80}, HAL_ERR_NACK);
    EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_ERR_I2C_NACK);
    EXPECT_EQ(hal_mock::BusCalls(), 2u);
    EXPECT_EQ(hal_mock::TotalDelayUs(), 0u);  // never reached the reset wait
}

TEST_F(Mpu6050Test, Req009InitDetectsConfigurationReadbackMismatch)
{
    hal_mock::ExpectWriteRead(kAddr, {0x75}, {0x68});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x80});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x01});
    hal_mock::ExpectWrite(kAddr, {0x19, 0x09, 0x03, 0x08, 0x08});
    hal_mock::ExpectWriteRead(kAddr, {0x19}, {0x09, 0x03, 0x08, 0x00});  // accel range lost
    hal_mock::ExpectWriteRead(kAddr, {0x6B}, {0x01});
    EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_ERR_CONFIG_VERIFY);
    mpu6050_raw_t raw{};
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_STATE);
}

TEST_F(Mpu6050Test, Req009InitDetectsSensorStillAsleep)
{
    hal_mock::ExpectWriteRead(kAddr, {0x75}, {0x68});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x80});
    hal_mock::ExpectWrite(kAddr, {0x6B, 0x01});
    hal_mock::ExpectWrite(kAddr, {0x19, 0x09, 0x03, 0x08, 0x08});
    hal_mock::ExpectWriteRead(kAddr, {0x19}, {0x09, 0x03, 0x08, 0x08});
    hal_mock::ExpectWriteRead(kAddr, {0x6B}, {0x40});  // SLEEP bit still set
    EXPECT_EQ(mpu6050_init(&dev_, &bus_, &cfg_), MPU6050_ERR_CONFIG_VERIFY);
}

TEST_F(Mpu6050Test, Req009InitRejectsInvalidParametersWithoutBusTraffic)
{
    EXPECT_EQ(mpu6050_init(nullptr, &bus_, &cfg_), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_init(&dev_, nullptr, &cfg_), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_init(&dev_, &bus_, nullptr), MPU6050_ERR_PARAM);

    std::vector<mpu6050_config_t> bad(7, cfg_);
    bad[0].address = 0x50;
    bad[1].accel_fs = static_cast<mpu6050_accel_fs_t>(4);
    bad[2].gyro_fs = static_cast<mpu6050_gyro_fs_t>(-1);
    bad[3].dlpf = static_cast<mpu6050_dlpf_t>(7);
    bad[4].i2c_timeout_us = 0;
    bad[5].i2c_timeout_us = 100001;
    bad[6].gyro_fs = static_cast<mpu6050_gyro_fs_t>(4);
    for (const mpu6050_config_t &c : bad) {
        EXPECT_EQ(mpu6050_init(&dev_, &bus_, &c), MPU6050_ERR_PARAM);
    }
    EXPECT_EQ(hal_mock::BusCalls(), 0u);
}

// --- Reading and conversion --------------------------------------------------

TEST_F(Mpu6050Test, Req009ReadParsesBigEndianFrameIntoSiUnits)
{
    InitDevice();
    ExpectFrame(kFrame);
    mpu6050_sample_t s{};
    ASSERT_EQ(mpu6050_read(&dev_, &s), MPU6050_OK);
    EXPECT_NEAR(s.accel_mps2.x, kG, 1e-5f);                       // +8192 LSB at 4 g
    EXPECT_NEAR(s.accel_mps2.y, -kG, 1e-5f);                      // -8192 LSB
    EXPECT_NEAR(s.accel_mps2.z, 32767.0f / 8192.0f * kG, 1e-4f);  // just under +4 g
    EXPECT_NEAR(s.gyro_rps.x, 10.0f * kDegToRad, 1e-6f);          // 655 LSB at 65.5 LSB/(deg/s)
    EXPECT_NEAR(s.gyro_rps.y, -10.0f * kDegToRad, 1e-6f);
    EXPECT_NEAR(s.gyro_rps.z, -32768.0f / 65.5f * kDegToRad, 1e-5f);
    EXPECT_NEAR(s.temp_c, -4260.0f / 340.0f + 36.53f, 1e-4f);
}

TEST_F(Mpu6050Test, Req009ReadDecodesTheSignedExtremes)
{
    InitDevice();
    ExpectFrame({0x80, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01, 0x00, 0x00});
    mpu6050_raw_t raw{};
    ASSERT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_OK);
    EXPECT_EQ(raw.accel[0], -32768);
    EXPECT_EQ(raw.accel[1], 32767);
    EXPECT_EQ(raw.accel[2], -1);
    EXPECT_EQ(raw.temp, 0);
    EXPECT_EQ(raw.gyro[0], 1);
    EXPECT_EQ(raw.gyro[1], -32767);
    EXPECT_EQ(raw.gyro[2], 0);
}

TEST_F(Mpu6050Test, Req009ReadReportsAndCountsEachBusErrorKind)
{
    InitDevice();
    ExpectFrame(kFrame, HAL_ERR_TIMEOUT);
    ExpectFrame(kFrame, HAL_ERR_NACK);
    ExpectFrame(kFrame, HAL_ERR_NACK);
    ExpectFrame(kFrame, HAL_ERR_BUS);
    ExpectFrame(kFrame);
    mpu6050_raw_t raw{};
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_I2C_TIMEOUT);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_I2C_NACK);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_I2C_NACK);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_I2C_BUS);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_OK);

    mpu6050_stats_t st{};
    ASSERT_EQ(mpu6050_get_stats(&dev_, &st), MPU6050_OK);
    EXPECT_EQ(st.reads, 5u);
    EXPECT_EQ(st.timeouts, 1u);
    EXPECT_EQ(st.nacks, 2u);
    EXPECT_EQ(st.bus_errors, 1u);
    EXPECT_EQ(st.bad_frames, 0u);
}

TEST_F(Mpu6050Test, Req009ReadFlagsImplausibleFramesAsBadData)
{
    InitDevice();
    ExpectFrame(std::vector<uint8_t>(14, 0xFF));  // SDA stuck high
    ExpectFrame(std::vector<uint8_t>(14, 0x00));  // SDA stuck low / sensor asleep
    std::vector<uint8_t> hot = kFrame;
    hot[6] = 0x40;  // temp raw 0x4000 = 16384 -> 84.7 degC: still in range
    hot[7] = 0x00;
    std::vector<uint8_t> too_hot = kFrame;
    too_hot[6] = 0x50;  // 0x5000 = 20480 -> 96.8 degC: out of range
    too_hot[7] = 0x00;
    ExpectFrame(hot);
    ExpectFrame(too_hot);

    mpu6050_raw_t raw{};
    raw.accel[0] = 1234;  // must survive the failed reads untouched
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_BAD_DATA);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_BAD_DATA);
    EXPECT_EQ(raw.accel[0], 1234);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_OK);
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_BAD_DATA);

    mpu6050_stats_t st{};
    ASSERT_EQ(mpu6050_get_stats(&dev_, &st), MPU6050_OK);
    EXPECT_EQ(st.bad_frames, 3u);
}

TEST_F(Mpu6050Test, Req009ConversionScalesEveryFullScaleRange)
{
    const float accel_lsb_per_g[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    const int16_t gyro_raw_for_100dps[] = {13100, 6550, 3280, 1640};
    for (int fs = 0; fs < 4; ++fs) {
        hal_mock::Reset();
        cfg_.accel_fs = static_cast<mpu6050_accel_fs_t>(fs);
        cfg_.gyro_fs = static_cast<mpu6050_gyro_fs_t>(fs);
        InitDevice();
        mpu6050_raw_t raw{};
        raw.accel[2] = static_cast<int16_t>(accel_lsb_per_g[fs]);  // exactly 1 g
        raw.gyro[0] = gyro_raw_for_100dps[fs];                      // 100 deg/s
        mpu6050_sample_t s{};
        ASSERT_EQ(mpu6050_convert(&dev_, &raw, &s), MPU6050_OK);
        EXPECT_NEAR(s.accel_mps2.z, kG, 1e-5f) << "fs=" << fs;
        EXPECT_NEAR(s.gyro_rps.x, 100.0f * kDegToRad, 1e-4f) << "fs=" << fs;
    }
}

TEST_F(Mpu6050Test, Req009CallsRequireInitAndValidPointers)
{
    mpu6050_raw_t raw{};
    mpu6050_sample_t s{};
    mpu6050_stats_t st{};
    EXPECT_EQ(mpu6050_read_raw(&dev_, &raw), MPU6050_ERR_STATE);
    EXPECT_EQ(mpu6050_convert(&dev_, &raw, &s), MPU6050_ERR_STATE);
    EXPECT_EQ(mpu6050_read(&dev_, &s), MPU6050_ERR_STATE);
    EXPECT_EQ(mpu6050_get_stats(&dev_, &st), MPU6050_ERR_STATE);

    InitDevice();
    EXPECT_EQ(mpu6050_read_raw(nullptr, &raw), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_read_raw(&dev_, nullptr), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_convert(nullptr, &raw, &s), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_convert(&dev_, nullptr, &s), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_convert(&dev_, &raw, nullptr), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_read(&dev_, nullptr), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_read(nullptr, &s), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_get_stats(nullptr, &st), MPU6050_ERR_PARAM);
    EXPECT_EQ(mpu6050_get_stats(&dev_, nullptr), MPU6050_ERR_PARAM);
    EXPECT_EQ(hal_mock::Remaining(), 0u);
}
