#include "mpu6050_model.h"

#include <math.h>
#include <stdbool.h>

#define REG_GYRO_CONFIG (0x1Bu)
#define REG_ACCEL_CONFIG (0x1Cu)
#define REG_DATA_FIRST (0x3Bu) /* ACCEL_XOUT_H */
#define REG_DATA_LAST (0x48u)  /* GYRO_ZOUT_L */
#define REG_PWR_MGMT_1 (0x6Bu)
#define REG_WHO_AM_I (0x75u)

#define PWR_RESET_BIT (0x80u)
#define PWR_SLEEP_BIT (0x40u)
#define WHO_AM_I_VALUE (0x68u)

#define MODEL_DEG_TO_RAD (0.0174532925199433f)

static const float k_accel_lsb_per_g[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float k_gyro_lsb_per_dps[4] = {131.0f, 65.5f, 32.8f, 16.4f};

static void reset_registers(mpu6050_model_t *m)
{
    for (uint32_t i = 0u; i < MPU6050_MODEL_NUM_REGS; ++i) {
        m->regs[i] = 0u;
    }
    m->regs[REG_PWR_MGMT_1] = PWR_SLEEP_BIT; /* the device powers up asleep */
    m->regs[REG_WHO_AM_I] = WHO_AM_I_VALUE;
    m->reg_ptr = 0u;
}

/* Round to the nearest count and saturate to the int16 range, as the ADC does. */
static int16_t quantise(float value, float lsb_per_unit)
{
    float counts = value * lsb_per_unit;
    if (!(counts > -32768.0f)) { /* also catches NaN */
        counts = -32768.0f;
    } else if (counts > 32767.0f) {
        counts = 32767.0f;
    } else {
        counts = roundf(counts);
    }
    return (int16_t)counts;
}

static void store_be16(uint8_t *dst, int16_t value)
{
    const uint16_t u = (uint16_t)value;
    dst[0] = (uint8_t)(u >> 8u);
    dst[1] = (uint8_t)(u & 0xFFu);
}

/* Refresh ACCEL_XOUT_H..GYRO_ZOUT_L from the physical inputs. */
static void update_data_registers(mpu6050_model_t *m)
{
    uint8_t *d = &m->regs[REG_DATA_FIRST];
    const bool asleep = (m->regs[REG_PWR_MGMT_1] & PWR_SLEEP_BIT) != 0u;

    if (asleep) {
        for (uint32_t i = 0u; i <= (REG_DATA_LAST - REG_DATA_FIRST); ++i) {
            d[i] = 0u; /* sensors are off: outputs read as zero */
        }
    } else {
        const uint32_t afs = ((uint32_t)m->regs[REG_ACCEL_CONFIG] >> 3u) & 0x3u;
        const uint32_t gfs = ((uint32_t)m->regs[REG_GYRO_CONFIG] >> 3u) & 0x3u;
        const float a_lsb = k_accel_lsb_per_g[afs] / ATT_GRAVITY_MPS2;   /* LSB per m/s^2 */
        const float g_lsb = k_gyro_lsb_per_dps[gfs] / MODEL_DEG_TO_RAD;  /* LSB per rad/s */
        store_be16(&d[0], quantise(m->accel_mps2.x, a_lsb));
        store_be16(&d[2], quantise(m->accel_mps2.y, a_lsb));
        store_be16(&d[4], quantise(m->accel_mps2.z, a_lsb));
        store_be16(&d[6], quantise(m->temp_c - 36.53f, 340.0f));
        store_be16(&d[8], quantise(m->gyro_rps.x, g_lsb));
        store_be16(&d[10], quantise(m->gyro_rps.y, g_lsb));
        store_be16(&d[12], quantise(m->gyro_rps.z, g_lsb));
    }
}

static hal_status_t fault_status(const mpu6050_model_t *m)
{
    hal_status_t status = HAL_OK;
    if (m->fault == MPU6050_MODEL_FAULT_TIMEOUT) {
        status = HAL_ERR_TIMEOUT;
    } else if (m->fault == MPU6050_MODEL_FAULT_NACK) {
        status = HAL_ERR_NACK;
    } else {
        status = HAL_OK;
    }
    return status;
}

void mpu6050_model_init(mpu6050_model_t *m, uint8_t address)
{
    if (m != NULL) {
        m->address = address;
        m->accel_mps2.x = 0.0f;
        m->accel_mps2.y = 0.0f;
        m->accel_mps2.z = ATT_GRAVITY_MPS2;
        m->gyro_rps.x = 0.0f;
        m->gyro_rps.y = 0.0f;
        m->gyro_rps.z = 0.0f;
        m->temp_c = 25.0f;
        m->fault = MPU6050_MODEL_FAULT_NONE;
        m->transactions = 0u;
        reset_registers(m);
    }
}

void mpu6050_model_set_inputs(mpu6050_model_t *m, const att_vec3_t *accel_mps2,
                              const att_vec3_t *gyro_rps, float temp_c)
{
    if ((m != NULL) && (accel_mps2 != NULL) && (gyro_rps != NULL)) {
        m->accel_mps2 = *accel_mps2;
        m->gyro_rps = *gyro_rps;
        m->temp_c = temp_c;
    }
}

void mpu6050_model_set_fault(mpu6050_model_t *m, mpu6050_model_fault_t fault)
{
    if (m != NULL) {
        m->fault = fault;
    }
}

hal_status_t mpu6050_model_write(mpu6050_model_t *m, const uint8_t *data, size_t len)
{
    hal_status_t status = HAL_OK;

    if ((m == NULL) || (data == NULL) || (len == 0u)) {
        status = HAL_ERR_PARAM;
    } else {
        m->transactions += 1u;
        status = fault_status(m);
    }

    if (status == HAL_OK) {
        m->reg_ptr = (uint8_t)(data[0] & 0x7Fu);
        for (size_t i = 1u; i < len; ++i) {
            const uint8_t reg = m->reg_ptr;
            if ((reg == REG_PWR_MGMT_1) && ((data[i] & PWR_RESET_BIT) != 0u)) {
                reset_registers(m); /* DEVICE_RESET: back to power-on state */
            } else if (reg != REG_WHO_AM_I) {
                m->regs[reg] = data[i];
            } else {
                /* WHO_AM_I is read-only */
            }
            m->reg_ptr = (uint8_t)((reg + 1u) & 0x7Fu);
        }
    }
    return status;
}

hal_status_t mpu6050_model_read(mpu6050_model_t *m, uint8_t *data, size_t len)
{
    hal_status_t status = HAL_OK;

    if ((m == NULL) || (data == NULL) || (len == 0u)) {
        status = HAL_ERR_PARAM;
    } else {
        m->transactions += 1u;
        status = fault_status(m);
    }

    if (status == HAL_OK) {
        update_data_registers(m);
        for (size_t i = 0u; i < len; ++i) {
            data[i] = (m->fault == MPU6050_MODEL_FAULT_BUS_HIGH) ? 0xFFu : m->regs[m->reg_ptr];
            m->reg_ptr = (uint8_t)((m->reg_ptr + 1u) & 0x7Fu);
        }
    }
    return status;
}
