/**
 * @file    FreeRTOSConfig.h
 * @brief   Kernel configuration for the Thunders Robotic mini-RTOS.
 *
 * The middleware implements the exact subset of the FreeRTOS API used by
 * this firmware (tasks, delays, queues, critical sections) on top of a
 * ~600 line Cortex-M4 kernel. For production, the official FreeRTOS kernel
 * can replace it without touching the application: this file only carries
 * the settings both implementations understand.
 */
#ifndef THUNDERS_FREERTOS_CONFIG_H
#define THUNDERS_FREERTOS_CONFIG_H

#include <stdint.h>

/* Scheduler */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configTICK_RATE_HZ                      ((TickType_t)1000U)
#define configMAX_PRIORITIES                    8U
#define configMINIMAL_STACK_SIZE                ((uint16_t)128U)
#define configMAX_TASK_NAME_LEN                 12U
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MUTEXES                       0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_TIMERS                        0
#define configUSE_TRACE_FACILITY                0

/* Static bump-allocator pool: 7 task stacks + queues */
#define configTOTAL_HEAP_SIZE                   ((size_t)(32U * 1024U))

/* Optional API inclusions used by the application */
#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    0
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1

/* Cortex-M specifics (handled by port.c) */
#define configKERNEL_INTERRUPT_PRIORITY         0xFFU  /* lowest */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x00U  /* PRIMASK model */
#define configPRIO_BITS                         4U
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15U

/* Hook required by some headers; unused here */
#define traceTASK_CREATE(x)      do { } while (0)
#define traceQUEUE_SEND(x)       do { } while (0)
#define traceQUEUE_RECEIVE(x)    do { } while (0)

#endif /* THUNDERS_FREERTOS_CONFIG_H */
