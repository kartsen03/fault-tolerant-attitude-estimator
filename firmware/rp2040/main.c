/*
 * RP2040 firmware entry point.
 *
 * Two statically allocated FreeRTOS tasks:
 *  - estimator (highest priority): every 10 ms, released by xTaskDelayUntil
 *    inside app_run_cycle(): read the MPU6050, update the estimate, and
 *    post every 10th result to a one-slot mailbox (never blocks).
 *  - telemetry (low priority): format the latest result and write it to the
 *    UART, plus a stack-usage line every 10 s. Slow UART output can never
 *    delay the estimator.
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "app.h"
#include "hal_rp2040.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "task.h"

#ifndef FTAE_VERSION
#define FTAE_VERSION "dev"
#endif
#ifndef FTAE_GIT_SHA
#define FTAE_GIT_SHA "unknown"
#endif

#define EST_TASK_PRIORITY (configMAX_PRIORITIES - 1u)
#define TLM_TASK_PRIORITY (tskIDLE_PRIORITY + 1u)
#define EST_STACK_WORDS (1024u)
#define TLM_STACK_WORDS (768u)
#define TELEMETRY_DIVIDER (10u)   /* 100 Hz loop, 10 Hz telemetry */
#define SYS_REPORT_CYCLES (100u)  /* telemetry lines between stack reports */

static app_t s_app;

static StaticTask_t s_est_tcb;
static StackType_t s_est_stack[EST_STACK_WORDS];
static TaskHandle_t s_est_task;
static StaticTask_t s_tlm_tcb;
static StackType_t s_tlm_stack[TLM_STACK_WORDS];
static TaskHandle_t s_tlm_task;

static StaticQueue_t s_tlm_queue_ctrl;
static uint8_t s_tlm_queue_storage[sizeof(app_output_t)];
static QueueHandle_t s_tlm_queue;

static void estimator_task(void *arg)
{
    app_t *app = (app_t *)arg;
    app_output_t out;
    for (;;) {
        (void)app_run_cycle(app, &out);
        if ((out.seq % TELEMETRY_DIVIDER) == 0u) {
            (void)xQueueOverwrite(s_tlm_queue, &out); /* mailbox: keep only the latest */
        }
    }
}

static void telemetry_task(void *arg)
{
    app_output_t out;
    char line[APP_TELEMETRY_MAX_LEN];
    uint32_t lines = 0u;
    (void)arg;
    for (;;) {
        if (xQueueReceive(s_tlm_queue, &out, portMAX_DELAY) == pdTRUE) {
            const size_t n = app_format_telemetry(&out, line, sizeof line);
            if (n > 0u) {
                hal_console_write(line, n);
            }
            lines += 1u;
            if ((lines % SYS_REPORT_CYCLES) == 0u) {
                /* Smallest free stack seen so far, in words. */
                printf("$SYS,%lu,est_stack_free=%lu,tlm_stack_free=%lu\r\n",
                       (unsigned long)(out.t_us / 1000000u),
                       (unsigned long)uxTaskGetStackHighWaterMark(s_est_task),
                       (unsigned long)uxTaskGetStackHighWaterMark(s_tlm_task));
            }
        }
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    panic("stack overflow in task %s", name);
}

void ftae_freertos_assert(const char *file, int line)
{
    panic("FreeRTOS assert at %s:%d", file, line);
}

int main(void)
{
    app_config_t cfg;

    stdio_init_all(); /* UART0, 115200 8N1 on GP0/GP1 */
    hal_rp2040_init();
    (void)app_default_config(&cfg);
    if (app_init(&s_app, hal_rp2040_i2c0(), &cfg) != ATT_OK) {
        panic("app_init failed");
    }
    printf("FTAE boot: fw %s (%s), MPU6050 init status %d\r\n", FTAE_VERSION, FTAE_GIT_SHA,
           (int)s_app.imu_status);

    s_tlm_queue = xQueueCreateStatic(1u, sizeof(app_output_t), s_tlm_queue_storage, &s_tlm_queue_ctrl);
    s_est_task = xTaskCreateStatic(estimator_task, "est", EST_STACK_WORDS, &s_app, EST_TASK_PRIORITY,
                                   s_est_stack, &s_est_tcb);
    s_tlm_task = xTaskCreateStatic(telemetry_task, "tlm", TLM_STACK_WORDS, NULL, TLM_TASK_PRIORITY,
                                   s_tlm_stack, &s_tlm_tcb);
    vTaskStartScheduler();
    panic("scheduler returned");
    return 0;
}
