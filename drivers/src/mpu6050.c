#include "drv/mpu6050.h"

#include <stdbool.h>
#include <stddef.h>

#define MPU6050_MAGIC (0x4D505530u) /* "MPU0" */
#define MPU6050_TIMEOUT_MAX_US (100000u)
#define MPU6050_CONFIG_BLOCK_LEN (4u) /* SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG */
#define MPU6050_FS_SHIFT (3u)         /* FS_SEL / AFS_SEL sit in bits 4:3 */

/* Temperature register range for the rated -40..+85 degC:
 * raw = (T - 36.53) * 340. A frame outside it is not a real reading. */
#define MPU6050_TEMP_RAW_MIN (-26020)
#define MPU6050_TEMP_RAW_MAX (16480)

#define MPU6050_DEG_TO_RAD (0.0174532925199433f)

/* Sensitivity per full-scale setting (datasheet section 6.1 and 6.2). */
static const float k_accel_lsb_per_g[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float k_gyro_lsb_per_dps[4] = {131.0f, 65.5f, 32.8f, 16.4f};

static void sat_inc(uint32_t *counter)
{
    if (*counter < UINT32_MAX) {
        *counter += 1u;
    }
}

static bool config_is_valid(const mpu6050_config_t *cfg)
{
    const bool addr_ok = (cfg->address == MPU6050_ADDR_AD0_LOW) ||
                         (cfg->address == MPU6050_ADDR_AD0_HIGH);
    /* Casting to unsigned also rejects negative values forced into the enums. */
    const bool accel_ok = (uint32_t)cfg->accel_fs <= (uint32_t)MPU6050_ACCEL_FS_16G;
    const bool gyro_ok = (uint32_t)cfg->gyro_fs <= (uint32_t)MPU6050_GYRO_FS_2000DPS;
    const bool dlpf_ok = (uint32_t)cfg->dlpf <= (uint32_t)MPU6050_DLPF_5HZ;
    const bool timeout_ok = (cfg->i2c_timeout_us > 0u) && (cfg->i2c_timeout_us <= MPU6050_TIMEOUT_MAX_US);
    return addr_ok && accel_ok && gyro_ok && dlpf_ok && timeout_ok;
}

static mpu6050_status_t map_hal_status(hal_status_t hal)
{
    mpu6050_status_t status;
    switch (hal) {
    case HAL_OK:
        status = MPU6050_OK;
        break;
    case HAL_ERR_TIMEOUT:
        status = MPU6050_ERR_I2C_TIMEOUT;
        break;
    case HAL_ERR_NACK:
        status = MPU6050_ERR_I2C_NACK;
        break;
    case HAL_ERR_PARAM:
    case HAL_ERR_BUS:
    default:
        status = MPU6050_ERR_I2C_BUS;
        break;
    }
    return status;
}

static void count_error(mpu6050_stats_t *stats, mpu6050_status_t status)
{
    if (status == MPU6050_ERR_I2C_TIMEOUT) {
        sat_inc(&stats->timeouts);
    } else if (status == MPU6050_ERR_I2C_NACK) {
        sat_inc(&stats->nacks);
    } else if (status == MPU6050_ERR_I2C_BUS) {
        sat_inc(&stats->bus_errors);
    } else if (status == MPU6050_ERR_BAD_DATA) {
        sat_inc(&stats->bad_frames);
    } else {
        /* success or a non-bus error: nothing to count */
    }
}

static mpu6050_status_t read_regs(hal_i2c_bus_t *bus, const mpu6050_config_t *cfg, uint8_t reg,
                                  uint8_t *buf, size_t len)
{
    const uint8_t reg_addr = reg;
    return map_hal_status(hal_i2c_write_read(bus, cfg->address, &reg_addr, 1u, buf, len, cfg->i2c_timeout_us));
}

static mpu6050_status_t write_regs(hal_i2c_bus_t *bus, const mpu6050_config_t *cfg,
                                   const uint8_t *data, size_t len)
{
    return map_hal_status(hal_i2c_write(bus, cfg->address, data, len, cfg->i2c_timeout_us));
}

/* Big-endian two's-complement pair to int16_t without implementation-defined
 * conversions: build the value in a wider signed type, then fold the sign. */
static int16_t be16_to_i16(uint8_t hi, uint8_t lo)
{
    const int32_t raw = (int32_t)(((uint32_t)hi << 8u) | (uint32_t)lo);
    return (int16_t)((raw > 32767) ? (raw - 65536) : raw);
}

static bool frame_is_implausible(const uint8_t frame[MPU6050_FRAME_LEN], int16_t temp_raw)
{
    bool all_zero = true;
    bool all_ones = true;
    for (uint32_t i = 0u; i < MPU6050_FRAME_LEN; ++i) {
        all_zero = all_zero && (frame[i] == 0x00u);
        all_ones = all_ones && (frame[i] == 0xFFu);
    }
    /* All 0xFF: SDA held high (device not driving the bus). All 0x00: SDA held
     * low, or the sensor is asleep / just reset. Both decode as plausible
     * temperatures, so they need their own check. */
    const bool temp_bad = (temp_raw < MPU6050_TEMP_RAW_MIN) || (temp_raw > MPU6050_TEMP_RAW_MAX);
    return all_zero || all_ones || temp_bad;
}

mpu6050_status_t mpu6050_default_config(mpu6050_config_t *cfg)
{
    mpu6050_status_t status = MPU6050_OK;
    if (cfg == NULL) {
        status = MPU6050_ERR_PARAM;
    } else {
        cfg->address = MPU6050_ADDR_AD0_LOW;
        cfg->accel_fs = MPU6050_ACCEL_FS_4G;
        cfg->gyro_fs = MPU6050_GYRO_FS_500DPS;
        cfg->dlpf = MPU6050_DLPF_44HZ; /* below the 50 Hz Nyquist limit of a 100 Hz loop */
        cfg->sample_rate_div = 9u;     /* 1 kHz / (1 + 9) = 100 Hz */
        cfg->i2c_timeout_us = 2000u;   /* a 14-byte read takes ~0.4 ms at 400 kHz, ~1.4 ms at 100 kHz */
    }
    return status;
}

mpu6050_status_t mpu6050_sample_rate_hz(const mpu6050_config_t *cfg, float *rate_hz)
{
    mpu6050_status_t status = MPU6050_OK;
    if ((cfg == NULL) || (rate_hz == NULL)) {
        status = MPU6050_ERR_PARAM;
    } else if (!config_is_valid(cfg)) {
        status = MPU6050_ERR_PARAM;
    } else {
        const float base_hz = (cfg->dlpf == MPU6050_DLPF_260HZ) ? 8000.0f : 1000.0f;
        *rate_hz = base_hz / (1.0f + (float)cfg->sample_rate_div);
    }
    return status;
}

mpu6050_status_t mpu6050_init(mpu6050_t *dev, hal_i2c_bus_t *bus, const mpu6050_config_t *cfg)
{
    mpu6050_status_t status = MPU6050_OK;
    uint8_t who_am_i = 0u;
    uint8_t pwr = 0u;
    uint8_t readback[MPU6050_CONFIG_BLOCK_LEN] = {0u, 0u, 0u, 0u};
    uint8_t block[1u + MPU6050_CONFIG_BLOCK_LEN] = {0u, 0u, 0u, 0u, 0u};

    if (dev != NULL) {
        dev->magic = 0u; /* stays invalid unless every step below succeeds */
    }

    if ((dev == NULL) || (bus == NULL) || (cfg == NULL)) {
        status = MPU6050_ERR_PARAM;
    } else if (!config_is_valid(cfg)) {
        status = MPU6050_ERR_PARAM;
    } else {
        status = read_regs(bus, cfg, MPU6050_REG_WHO_AM_I, &who_am_i, 1u);
        if ((status == MPU6050_OK) && (who_am_i != MPU6050_WHO_AM_I_VALUE)) {
            status = MPU6050_ERR_WHO_AM_I;
        }
    }

    if (status == MPU6050_OK) {
        const uint8_t reset[2] = {MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_RESET};
        status = write_regs(bus, cfg, reset, sizeof reset);
    }
    if (status == MPU6050_OK) {
        const uint8_t wake[2] = {MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLK_PLL_X};
        hal_delay_us(MPU6050_RESET_DELAY_US);
        status = write_regs(bus, cfg, wake, sizeof wake);
    }
    if (status == MPU6050_OK) {
        /* SMPLRT_DIV..ACCEL_CONFIG are consecutive: one burst write. */
        block[0] = MPU6050_REG_SMPLRT_DIV;
        block[1] = cfg->sample_rate_div;
        block[2] = (uint8_t)cfg->dlpf;
        block[3] = (uint8_t)((uint32_t)cfg->gyro_fs << MPU6050_FS_SHIFT);
        block[4] = (uint8_t)((uint32_t)cfg->accel_fs << MPU6050_FS_SHIFT);
        status = write_regs(bus, cfg, block, sizeof block);
    }
    if (status == MPU6050_OK) {
        status = read_regs(bus, cfg, MPU6050_REG_SMPLRT_DIV, readback, sizeof readback);
    }
    if (status == MPU6050_OK) {
        status = read_regs(bus, cfg, MPU6050_REG_PWR_MGMT_1, &pwr, 1u);
    }
    if (status == MPU6050_OK) {
        bool same = (pwr == MPU6050_PWR_CLK_PLL_X);
        for (uint32_t i = 0u; i < MPU6050_CONFIG_BLOCK_LEN; ++i) {
            same = same && (readback[i] == block[i + 1u]);
        }
        if (!same) {
            status = MPU6050_ERR_CONFIG_VERIFY;
        }
    }
    if (status == MPU6050_OK) {
        dev->bus = bus;
        dev->cfg = *cfg;
        dev->accel_scale = ATT_GRAVITY_MPS2 / k_accel_lsb_per_g[(uint32_t)cfg->accel_fs];
        dev->gyro_scale = MPU6050_DEG_TO_RAD / k_gyro_lsb_per_dps[(uint32_t)cfg->gyro_fs];
        dev->stats.reads = 0u;
        dev->stats.timeouts = 0u;
        dev->stats.nacks = 0u;
        dev->stats.bus_errors = 0u;
        dev->stats.bad_frames = 0u;
        dev->magic = MPU6050_MAGIC;
    }
    return status;
}

mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    mpu6050_status_t status = MPU6050_OK;
    uint8_t frame[MPU6050_FRAME_LEN] = {0u};

    if ((dev == NULL) || (raw == NULL)) {
        status = MPU6050_ERR_PARAM;
    } else if (dev->magic != MPU6050_MAGIC) {
        status = MPU6050_ERR_STATE;
    } else {
        sat_inc(&dev->stats.reads);
        status = read_regs(dev->bus, &dev->cfg, MPU6050_REG_ACCEL_XOUT_H, frame, sizeof frame);
        if (status == MPU6050_OK) {
            mpu6050_raw_t parsed;
            parsed.accel[0] = be16_to_i16(frame[0], frame[1]);
            parsed.accel[1] = be16_to_i16(frame[2], frame[3]);
            parsed.accel[2] = be16_to_i16(frame[4], frame[5]);
            parsed.temp = be16_to_i16(frame[6], frame[7]);
            parsed.gyro[0] = be16_to_i16(frame[8], frame[9]);
            parsed.gyro[1] = be16_to_i16(frame[10], frame[11]);
            parsed.gyro[2] = be16_to_i16(frame[12], frame[13]);
            if (frame_is_implausible(frame, parsed.temp)) {
                status = MPU6050_ERR_BAD_DATA;
            } else {
                *raw = parsed;
            }
        }
        count_error(&dev->stats, status);
    }
    return status;
}

mpu6050_status_t mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw,
                                 mpu6050_sample_t *out)
{
    mpu6050_status_t status = MPU6050_OK;
    if ((dev == NULL) || (raw == NULL) || (out == NULL)) {
        status = MPU6050_ERR_PARAM;
    } else if (dev->magic != MPU6050_MAGIC) {
        status = MPU6050_ERR_STATE;
    } else {
        out->accel_mps2.x = (float)raw->accel[0] * dev->accel_scale;
        out->accel_mps2.y = (float)raw->accel[1] * dev->accel_scale;
        out->accel_mps2.z = (float)raw->accel[2] * dev->accel_scale;
        out->gyro_rps.x = (float)raw->gyro[0] * dev->gyro_scale;
        out->gyro_rps.y = (float)raw->gyro[1] * dev->gyro_scale;
        out->gyro_rps.z = (float)raw->gyro[2] * dev->gyro_scale;
        out->temp_c = ((float)raw->temp / 340.0f) + 36.53f;
    }
    return status;
}

mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_sample_t *out)
{
    mpu6050_status_t status = MPU6050_OK;
    mpu6050_raw_t raw;
    if (out == NULL) {
        status = MPU6050_ERR_PARAM;
    } else {
        status = mpu6050_read_raw(dev, &raw);
    }
    if (status == MPU6050_OK) {
        status = mpu6050_convert(dev, &raw, out);
    }
    return status;
}

mpu6050_status_t mpu6050_get_stats(const mpu6050_t *dev, mpu6050_stats_t *out)
{
    mpu6050_status_t status = MPU6050_OK;
    if ((dev == NULL) || (out == NULL)) {
        status = MPU6050_ERR_PARAM;
    } else if (dev->magic != MPU6050_MAGIC) {
        status = MPU6050_ERR_STATE;
    } else {
        *out = dev->stats;
    }
    return status;
}
