#include "hal_host.h"

#include <stdbool.h>

/* Peripherals are singletons, so the host HAL keeps its clock and console
 * sink in one file-scope struct (the core and drivers have no globals). */
typedef struct {
    uint32_t now_us;
    hal_host_console_fn console;
    void *console_ctx;
} hal_host_state_t;

static hal_host_state_t s_host = {0u, NULL, NULL};

#define HALF_RANGE_U32 (0x80000000u)

void hal_host_reset(uint32_t start_us)
{
    s_host.now_us = start_us;
    s_host.console = NULL;
    s_host.console_ctx = NULL;
}

void hal_host_advance_us(uint32_t us)
{
    s_host.now_us += us;
}

void hal_host_set_console(hal_host_console_fn fn, void *ctx)
{
    s_host.console = fn;
    s_host.console_ctx = ctx;
}

static hal_status_t check_target(const hal_i2c_bus_t *bus, uint8_t addr7)
{
    hal_status_t status = HAL_OK;
    if ((bus->imu == NULL) || (bus->imu->address != addr7)) {
        status = HAL_ERR_NACK; /* nobody answers at this address */
    }
    return status;
}

hal_status_t hal_i2c_write(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *data, size_t len,
                           uint32_t timeout_us)
{
    hal_status_t status = HAL_OK;
    if ((bus == NULL) || (data == NULL) || (len == 0u) || (timeout_us == 0u)) {
        status = HAL_ERR_PARAM;
    } else {
        status = check_target(bus, addr7);
    }
    if (status == HAL_OK) {
        status = mpu6050_model_write(bus->imu, data, len);
    }
    return status;
}

hal_status_t hal_i2c_write_read(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *wdata,
                                size_t wlen, uint8_t *rdata, size_t rlen, uint32_t timeout_us)
{
    hal_status_t status = HAL_OK;
    if ((bus == NULL) || (wdata == NULL) || (wlen == 0u) || (rdata == NULL) || (rlen == 0u) ||
        (timeout_us == 0u)) {
        status = HAL_ERR_PARAM;
    } else {
        status = check_target(bus, addr7);
    }
    if (status == HAL_OK) {
        status = mpu6050_model_write(bus->imu, wdata, wlen);
    }
    if (status == HAL_OK) {
        status = mpu6050_model_read(bus->imu, rdata, rlen);
    }
    return status;
}

uint32_t hal_time_us(void)
{
    return s_host.now_us;
}

void hal_delay_us(uint32_t us)
{
    s_host.now_us += us;
}

hal_status_t hal_periodic_init(hal_periodic_t *p, uint32_t period_us)
{
    hal_status_t status = HAL_OK;
    if ((p == NULL) || (period_us == 0u) || (period_us >= HALF_RANGE_U32)) {
        status = HAL_ERR_PARAM;
    } else {
        p->period_us = period_us;
        p->last_release = s_host.now_us;
        p->started = true;
    }
    return status;
}

/* Same contract as xTaskDelayUntil: the next release is last + period. If
 * that moment has already passed, return at once and report the miss; the
 * schedule itself never shifts, so late cycles do not accumulate drift. */
hal_status_t hal_periodic_wait(hal_periodic_t *p, bool *missed)
{
    hal_status_t status = HAL_OK;
    if ((p == NULL) || (missed == NULL) || (!p->started)) {
        status = HAL_ERR_PARAM;
    } else {
        const uint32_t next = p->last_release + p->period_us;
        const uint32_t since = s_host.now_us - next; /* modular: "now - next" */
        const bool late = (since != 0u) && (since < HALF_RANGE_U32);
        if (late) {
            *missed = true;
        } else {
            *missed = false;
            s_host.now_us = next;
        }
        p->last_release = next;
    }
    return status;
}

void hal_console_write(const char *data, size_t len)
{
    if ((s_host.console != NULL) && (data != NULL)) {
        s_host.console(data, len, s_host.console_ctx);
    }
}
