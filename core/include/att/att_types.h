/**
 * @file att_types.h
 * @brief Types and limits shared by every module of the attitude core.
 *
 * Conventions used throughout the project:
 *  - Units are SI: m/s^2 for specific force, rad/s for angular rate,
 *    rad for angles, seconds for durations, microseconds for timestamps.
 *  - The sensor frame is the body frame, right-handed. With the board level
 *    and at rest the accelerometer reads +1 g on +z (z points up).
 *  - Euler angles use the Z-Y-X (yaw, pitch, roll) sequence. Positive angles
 *    are right-hand rotations about the body axes.
 *  - Roll is reported in (-pi, pi]; pitch in [-pi/2, pi/2].
 */
#ifndef ATT_TYPES_H
#define ATT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Status returned by every public core function. */
typedef enum {
    ATT_OK = 0,            /**< success */
    ATT_ERR_NULL = 1,      /**< a required pointer argument was NULL */
    ATT_ERR_RANGE = 2,     /**< an argument was non-finite or out of range */
    ATT_ERR_STATE = 3,     /**< object not initialised, or not ready */
    ATT_ERR_TIMESTAMP = 4  /**< sample time not monotonic or step out of range */
} att_status_t;

/** Three-axis vector in the sensor (body) frame. */
typedef struct {
    float x;
    float y;
    float z;
} att_vec3_t;

/** Roll and pitch in radians. */
typedef struct {
    float roll_rad;
    float pitch_rad;
} att_euler_t;

/** Standard gravity [m/s^2]. */
#define ATT_GRAVITY_MPS2 (9.80665f)

/**
 * Input-validation bounds. Values beyond these are rejected as
 * ATT_ERR_RANGE before any arithmetic, which keeps every intermediate
 * result finite. They are far outside what the MPU6050 can report
 * (16 g, 2000 deg/s), so they never reject a genuine reading; physical
 * plausibility is the fault monitor's job, not the input checks'.
 */
#define ATT_ACCEL_ABS_MAX_MPS2 (1000.0f)
#define ATT_GYRO_ABS_MAX_RPS (100.0f)

#ifdef __cplusplus
}
#endif

#endif /* ATT_TYPES_H */
