/**
 * @file mpu6050_model.h
 * @brief Register-level MPU6050 model for the host simulator HAL.
 *
 * Behaves like the device on the bus: WHO_AM_I, power-on sleep, reset via
 * PWR_MGMT_1, auto-incrementing register pointer, and data registers that
 * quantise and saturate the physical inputs according to the full-scale
 * ranges the driver wrote. Bus-level faults can be injected per sample.
 */
#ifndef MPU6050_MODEL_H
#define MPU6050_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "att/att_types.h"
#include "hal/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_MODEL_NUM_REGS (128u)

typedef enum {
    MPU6050_MODEL_FAULT_NONE = 0,
    MPU6050_MODEL_FAULT_TIMEOUT = 1, /**< every transaction times out */
    MPU6050_MODEL_FAULT_NACK = 2,    /**< device does not acknowledge */
    MPU6050_MODEL_FAULT_BUS_HIGH = 3 /**< reads return 0xFF (SDA stuck high) */
} mpu6050_model_fault_t;

typedef struct {
    uint8_t address;
    uint8_t regs[MPU6050_MODEL_NUM_REGS];
    uint8_t reg_ptr;
    att_vec3_t accel_mps2; /**< physical specific force presented to the sensor */
    att_vec3_t gyro_rps;   /**< physical angular rate presented to the sensor */
    float temp_c;
    mpu6050_model_fault_t fault;
    uint32_t transactions;
} mpu6050_model_t;

/** Power-on state: asleep, registers at reset values, level and at rest. */
void mpu6050_model_init(mpu6050_model_t *m, uint8_t address);

/** Set the physical inputs used by the next data-register read. */
void mpu6050_model_set_inputs(mpu6050_model_t *m, const att_vec3_t *accel_mps2,
                              const att_vec3_t *gyro_rps, float temp_c);

/** Inject (or clear) a bus-level fault for subsequent transactions. */
void mpu6050_model_set_fault(mpu6050_model_t *m, mpu6050_model_fault_t fault);

/** Bus write: first byte selects the register, the rest are written in order. */
hal_status_t mpu6050_model_write(mpu6050_model_t *m, const uint8_t *data, size_t len);

/** Bus read from the current register pointer, auto-incrementing. */
hal_status_t mpu6050_model_read(mpu6050_model_t *m, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_MODEL_H */
