/**
 * @file mpu6050.h
 * @brief MPU6050 accelerometer + gyroscope driver on top of the HAL.
 *
 * Covers the WHO_AM_I check, configuration (sample rate, full-scale ranges,
 * digital low-pass filter) with read-back verification, 14-byte burst
 * reads of accelerometer/temperature/gyroscope, conversion to SI units,
 * and explicit reporting of I2C timeouts, NACKs and implausible frames.
 *
 * Every call performs a bounded number of bus transactions, each limited
 * by the configured timeout, and never retries internally: retry and
 * fallback policy belongs to the caller (the fault monitor counts dropouts).
 */
#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include <stdint.h>

#include "att/att_types.h"
#include "hal/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_AD0_LOW (0x68u)
#define MPU6050_ADDR_AD0_HIGH (0x69u)
#define MPU6050_WHO_AM_I_VALUE (0x68u)

/* Register map (MPU-6000/6050 Register Map, rev 4.2). */
#define MPU6050_REG_SMPLRT_DIV (0x19u)
#define MPU6050_REG_CONFIG (0x1Au)
#define MPU6050_REG_GYRO_CONFIG (0x1Bu)
#define MPU6050_REG_ACCEL_CONFIG (0x1Cu)
#define MPU6050_REG_ACCEL_XOUT_H (0x3Bu)
#define MPU6050_REG_PWR_MGMT_1 (0x6Bu)
#define MPU6050_REG_WHO_AM_I (0x75u)

#define MPU6050_PWR_RESET (0x80u)      /* DEVICE_RESET */
#define MPU6050_PWR_CLK_PLL_X (0x01u)  /* wake, clock from X-gyro PLL */
#define MPU6050_RESET_DELAY_US (100000u)
#define MPU6050_FRAME_LEN (14u)        /* accel(6) + temp(2) + gyro(6) */

typedef enum {
    MPU6050_ACCEL_FS_2G = 0,
    MPU6050_ACCEL_FS_4G = 1,
    MPU6050_ACCEL_FS_8G = 2,
    MPU6050_ACCEL_FS_16G = 3
} mpu6050_accel_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS = 0,
    MPU6050_GYRO_FS_500DPS = 1,
    MPU6050_GYRO_FS_1000DPS = 2,
    MPU6050_GYRO_FS_2000DPS = 3
} mpu6050_gyro_fs_t;

/** DLPF_CFG: accelerometer / gyroscope bandwidth. */
typedef enum {
    MPU6050_DLPF_260HZ = 0, /* gyro output rate 8 kHz in this setting */
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ = 2,
    MPU6050_DLPF_44HZ = 3,
    MPU6050_DLPF_21HZ = 4,
    MPU6050_DLPF_10HZ = 5,
    MPU6050_DLPF_5HZ = 6
} mpu6050_dlpf_t;

typedef struct {
    uint8_t address;             /**< MPU6050_ADDR_AD0_LOW or _HIGH */
    mpu6050_accel_fs_t accel_fs; /**< accelerometer full scale */
    mpu6050_gyro_fs_t gyro_fs;   /**< gyroscope full scale */
    mpu6050_dlpf_t dlpf;         /**< digital low-pass filter */
    uint8_t sample_rate_div;     /**< rate = base / (1 + div); base 1 kHz, 8 kHz if DLPF 260 Hz */
    uint32_t i2c_timeout_us;     /**< per bus phase, 1..100000 */
} mpu6050_config_t;

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_PARAM = 1,         /**< NULL pointer or invalid configuration */
    MPU6050_ERR_STATE = 2,         /**< device not initialised */
    MPU6050_ERR_I2C_TIMEOUT = 3,   /**< bus transaction timed out */
    MPU6050_ERR_I2C_NACK = 4,      /**< no acknowledge: device absent or busy */
    MPU6050_ERR_I2C_BUS = 5,       /**< other bus error */
    MPU6050_ERR_WHO_AM_I = 6,      /**< wrong device identity */
    MPU6050_ERR_CONFIG_VERIFY = 7, /**< read-back differs from what was written */
    MPU6050_ERR_BAD_DATA = 8       /**< implausible frame (all 0x00/0xFF, temperature out of range) */
} mpu6050_status_t;

/** Raw register values from one burst read. */
typedef struct {
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} mpu6050_raw_t;

/** One sample in SI units. */
typedef struct {
    att_vec3_t accel_mps2;
    att_vec3_t gyro_rps;
    float temp_c;
} mpu6050_sample_t;

/** Error counters, for health reporting. */
typedef struct {
    uint32_t reads;
    uint32_t timeouts;
    uint32_t nacks;
    uint32_t bus_errors;
    uint32_t bad_frames;
} mpu6050_stats_t;

/** Driver instance. All state is here; no globals. */
typedef struct {
    uint32_t magic;
    hal_i2c_bus_t *bus;
    mpu6050_config_t cfg;
    float accel_scale; /**< m/s^2 per LSB */
    float gyro_scale;  /**< rad/s per LSB */
    mpu6050_stats_t stats;
} mpu6050_t;

/** Recommended configuration: 0x68, +-4 g, +-500 deg/s, DLPF 44 Hz, 100 Hz, 2 ms timeout. */
mpu6050_status_t mpu6050_default_config(mpu6050_config_t *cfg);

/** Output data rate implied by a configuration [Hz]. */
mpu6050_status_t mpu6050_sample_rate_hz(const mpu6050_config_t *cfg, float *rate_hz);

/**
 * Check WHO_AM_I, reset, wake, configure and verify by read-back.
 * Blocks for MPU6050_RESET_DELAY_US. On failure dev is left uninitialised.
 */
mpu6050_status_t mpu6050_init(mpu6050_t *dev, hal_i2c_bus_t *bus, const mpu6050_config_t *cfg);

/** One 14-byte burst read starting at ACCEL_XOUT_H. */
mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);

/** Convert raw register values to SI units using the configured ranges. */
mpu6050_status_t mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw,
                                 mpu6050_sample_t *out);

/** mpu6050_read_raw() followed by mpu6050_convert(). */
mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_sample_t *out);

/** Copy the error counters. */
mpu6050_status_t mpu6050_get_stats(const mpu6050_t *dev, mpu6050_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DRV_MPU6050_H */
