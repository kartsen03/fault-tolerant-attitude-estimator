#include "app.h"

#include <stddef.h>

#define APP_MAGIC (0x41505030u) /* "APP0" */

static void sat_inc(uint32_t *counter)
{
    if (*counter < UINT32_MAX) {
        *counter += 1u;
    }
}

att_status_t app_default_config(app_config_t *cfg)
{
    att_status_t status = ATT_OK;
    if (cfg == NULL) {
        status = ATT_ERR_NULL;
    } else {
        (void)mpu6050_default_config(&cfg->imu);
        status = att_est_default_config(&cfg->est);
        cfg->period_us = APP_PERIOD_US;
    }
    return status;
}

att_status_t app_init(app_t *app, hal_i2c_bus_t *bus, const app_config_t *cfg)
{
    att_status_t status = ATT_OK;

    if ((app == NULL) || (bus == NULL) || (cfg == NULL)) {
        status = ATT_ERR_NULL;
    } else if (cfg->period_us == 0u) {
        status = ATT_ERR_RANGE;
    } else {
        app->magic = 0u;
        status = att_est_init(&app->est, &cfg->est);
    }

    if (status == ATT_OK) {
        app->cfg = *cfg;
        app->seq = 0u;
        app->overruns = 0u;
        app->read_errors = 0u;
        /* Sensor problems are reported every cycle rather than stopping the app. */
        app->imu_status = mpu6050_init(&app->imu, bus, &cfg->imu);
        if (hal_periodic_init(&app->sched, cfg->period_us) != HAL_OK) {
            status = ATT_ERR_STATE;
        } else {
            app->magic = APP_MAGIC;
        }
    }
    return status;
}

att_status_t app_run_cycle(app_t *app, app_output_t *out)
{
    att_status_t status = ATT_OK;

    if ((app == NULL) || (out == NULL)) {
        status = ATT_ERR_NULL;
    } else if (app->magic != APP_MAGIC) {
        status = ATT_ERR_STATE;
    } else {
        bool missed = false;
        mpu6050_sample_t reading = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.0f};
        mpu6050_status_t read_status = app->imu_status;
        att_imu_sample_t sample;

        /* Block until the next 10 ms release (vTaskDelayUntil on the target). */
        (void)hal_periodic_wait(&app->sched, &missed);
        if (missed) {
            sat_inc(&app->overruns);
        }

        sample.t_us = hal_time_us();
        if (app->imu_status == MPU6050_OK) {
            read_status = mpu6050_read(&app->imu, &reading);
        }
        if (read_status != MPU6050_OK) {
            sat_inc(&app->read_errors);
        }
        sample.accel_mps2 = reading.accel_mps2;
        sample.gyro_rps = reading.gyro_rps;
        sample.valid = (read_status == MPU6050_OK);

        /* Rejections are visible in out->est (faults, mode, valid). */
        (void)att_est_update(&app->est, &sample, &out->est);

        out->seq = app->seq;
        out->t_us = sample.t_us;
        out->read_status = read_status;
        out->overruns = app->overruns;
        out->read_errors = app->read_errors;
        app->seq += 1u; /* wraps after ~497 days at 100 Hz */
    }
    return status;
}
