#include "hal_rp2040.h"

#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/time.h"
#include "task.h"

#define HAL_US_PER_TICK (1000000u / (uint32_t)configTICK_RATE_HZ)

struct hal_i2c_bus {
    i2c_inst_t *inst;
};

static struct hal_i2c_bus s_i2c0 = {NULL};

void hal_rp2040_init(void)
{
    (void)i2c_init(i2c0, HAL_RP2040_I2C_HZ);
    gpio_set_function(HAL_RP2040_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(HAL_RP2040_SCL_PIN, GPIO_FUNC_I2C);
    /* Weak internal pull-ups; MPU6050 breakouts also carry their own. */
    gpio_pull_up(HAL_RP2040_SDA_PIN);
    gpio_pull_up(HAL_RP2040_SCL_PIN);
    s_i2c0.inst = i2c0;
}

hal_i2c_bus_t *hal_rp2040_i2c0(void)
{
    return &s_i2c0;
}

/* Pico SDK return convention: byte count on success, PICO_ERROR_GENERIC when
 * the address is not acknowledged, PICO_ERROR_TIMEOUT on timeout. */
static hal_status_t map_result(int rc, size_t expected)
{
    hal_status_t status;
    if (rc == PICO_ERROR_TIMEOUT) {
        status = HAL_ERR_TIMEOUT;
    } else if (rc == PICO_ERROR_GENERIC) {
        status = HAL_ERR_NACK;
    } else if ((rc < 0) || ((size_t)rc != expected)) {
        status = HAL_ERR_BUS;
    } else {
        status = HAL_OK;
    }
    return status;
}

hal_status_t hal_i2c_write(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *data, size_t len,
                           uint32_t timeout_us)
{
    hal_status_t status = HAL_ERR_PARAM;
    if ((bus != NULL) && (bus->inst != NULL) && (data != NULL) && (len > 0u) && (timeout_us > 0u)) {
        status = map_result(i2c_write_timeout_us(bus->inst, addr7, data, len, false, timeout_us), len);
    }
    return status;
}

hal_status_t hal_i2c_write_read(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *wdata,
                                size_t wlen, uint8_t *rdata, size_t rlen, uint32_t timeout_us)
{
    hal_status_t status = HAL_ERR_PARAM;
    if ((bus != NULL) && (bus->inst != NULL) && (wdata != NULL) && (wlen > 0u) && (rdata != NULL) &&
        (rlen > 0u) && (timeout_us > 0u)) {
        /* nostop = true: repeated START between the register write and the read. */
        status = map_result(i2c_write_timeout_us(bus->inst, addr7, wdata, wlen, true, timeout_us), wlen);
        if (status == HAL_OK) {
            status = map_result(i2c_read_timeout_us(bus->inst, addr7, rdata, rlen, false, timeout_us), rlen);
        }
    }
    return status;
}

uint32_t hal_time_us(void)
{
    return time_us_32();
}

void hal_delay_us(uint32_t us)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        /* Round up so the task sleeps at least the requested time. */
        const TickType_t ticks = (TickType_t)((us + HAL_US_PER_TICK - 1u) / HAL_US_PER_TICK);
        vTaskDelay(ticks);
    } else {
        busy_wait_us_32(us);
    }
}

hal_status_t hal_periodic_init(hal_periodic_t *p, uint32_t period_us)
{
    hal_status_t status = HAL_ERR_PARAM;
    /* The period must be a whole number of RTOS ticks (1 ms at 1 kHz). */
    if ((p != NULL) && (period_us >= HAL_US_PER_TICK) && ((period_us % HAL_US_PER_TICK) == 0u)) {
        p->period_us = period_us;
        p->last_release = (uint32_t)xTaskGetTickCount();
        p->started = true;
        status = HAL_OK;
    }
    return status;
}

hal_status_t hal_periodic_wait(hal_periodic_t *p, bool *missed)
{
    hal_status_t status = HAL_ERR_PARAM;
    if ((p != NULL) && (missed != NULL) && (p->started)) {
        TickType_t last = (TickType_t)p->last_release;
        /* xTaskDelayUntil is vTaskDelayUntil plus a return value: the next
         * wake time is last + period (an absolute schedule), and pdFALSE
         * means that time had already passed, i.e. the previous cycle overran. */
        const BaseType_t delayed = xTaskDelayUntil(&last, (TickType_t)(p->period_us / HAL_US_PER_TICK));
        p->last_release = (uint32_t)last;
        *missed = (delayed == pdFALSE);
        status = HAL_OK;
    }
    return status;
}

void hal_console_write(const char *data, size_t len)
{
    if ((data != NULL) && (len > 0u)) {
        uart_write_blocking(uart0, (const uint8_t *)data, len);
    }
}
