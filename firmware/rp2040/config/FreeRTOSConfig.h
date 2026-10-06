/*
 * FreeRTOS configuration for the RP2040 firmware.
 *
 * Single core (Wokwi simulates one RP2040 core, and one core keeps the
 * schedule simple to reason about), 1 kHz tick, preemptive priorities,
 * static allocation only: no heap implementation is linked, so any
 * dynamic RTOS allocation would fail at link time.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler */
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 1
#define configUSE_TICKLESS_IDLE 0
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES 8
#define configMINIMAL_STACK_SIZE ((configSTACK_DEPTH_TYPE)256)
#define configSTACK_DEPTH_TYPE uint32_t
#define configIDLE_SHOULD_YIELD 1
#define configNUMBER_OF_CORES 1

/* Memory: static only */
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configKERNEL_PROVIDED_STATIC_MEMORY 1

/* Features */
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_QUEUE_SETS 0
#define configQUEUE_REGISTRY_SIZE 0
/* The timer daemon is required by the RP2040 port's SDK-sync interop, which
 * pends event-group updates to it (xEventGroupSetBitsFromISR). It runs one
 * priority below the estimator task, so it can never preempt the 100 Hz loop. */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define configTIMER_QUEUE_LENGTH 8
#define configTIMER_TASK_STACK_DEPTH 512
#define configUSE_CO_ROUTINES 0
#define configUSE_TRACE_FACILITY 0
#define configGENERATE_RUN_TIME_STATS 0
#define configUSE_NEWLIB_REENTRANT 0
#define configENABLE_BACKWARD_COMPATIBILITY 0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configMESSAGE_BUFFER_LENGTH_TYPE size_t

/* Hooks */
#define configUSE_IDLE_HOOK 0
#define configUSE_PASSIVE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_MALLOC_FAILED_HOOK 0
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0
#define configCHECK_FOR_STACK_OVERFLOW 2

/* RP2040 port: interoperate with SDK sync/time primitives */
#define configSUPPORT_PICO_SYNC_INTEROP 1
#define configSUPPORT_PICO_TIME_INTEROP 1

/* Assertions stay enabled in release builds. */
void ftae_freertos_assert(const char *file, int line);
#define configASSERT(x)                                                                            \
    do {                                                                                           \
        if ((x) == 0) {                                                                            \
            ftae_freertos_assert(__FILE__, __LINE__);                                              \
        }                                                                                          \
    } while (0)

/* API functions included in the build */
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskDelayUntil 1 /* also enables the vTaskDelayUntil macro */
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_vTaskPrioritySet 0
#define INCLUDE_uxTaskPriorityGet 0
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xTimerPendFunctionCall 1

#endif /* FREERTOS_CONFIG_H */
