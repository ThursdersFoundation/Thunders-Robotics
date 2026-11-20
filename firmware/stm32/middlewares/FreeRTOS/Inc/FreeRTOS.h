/**
 * @file    FreeRTOS.h
 * @brief   Common kernel definitions (types, status codes, port macros).
 */
#ifndef THUNDERS_FREERTOS_H
#define THUNDERS_FREERTOS_H

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOSConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Basic types                                                         */
/* ------------------------------------------------------------------ */
typedef long              BaseType_t;
typedef unsigned long     UBaseType_t;
typedef uint32_t          TickType_t;
typedef uint32_t          StackType_t;

#define pdTRUE       ((BaseType_t)1)
#define pdFALSE      ((BaseType_t)0)
#define pdPASS       pdTRUE
#define pdFAIL       pdFALSE

#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define portBYTE_ALIGNMENT 8U

#define pdMS_TO_TICKS(ms)  (((TickType_t)(ms) * (TickType_t)configTICK_RATE_HZ) / (TickType_t)1000U)

/* Queue status codes */
#define errQUEUE_FULL   ((BaseType_t)-1)
#define errQUEUE_EMPTY  ((BaseType_t)-2)

/* ------------------------------------------------------------------ */
/* Critical sections: PRIMASK with nesting counter (port.c)            */
/* ------------------------------------------------------------------ */
void vPortEnterCritical(void);
void vPortExitCritical(void);
#define taskENTER_CRITICAL()        vPortEnterCritical()
#define taskEXIT_CRITICAL()         vPortExitCritical()
#define taskDISABLE_INTERRUPTS()    vPortEnterCritical()
#define taskENABLE_INTERRUPTS()     vPortExitCritical()

/* Yield: pend PendSV (port.c) */
void vPortYield(void);
#define portYIELD()              vPortYield()
#define portYIELD_FROM_ISR(x)    do { if (x) { vPortYield(); } } while (0)

/* Memory: static bump allocator (port.c) */
void *pvPortMalloc(size_t xSize);
void  vPortFree(void *pv);

/* ------------------------------------------------------------------ */
/* Scheduler state (tasks.c)                                           */
/* ------------------------------------------------------------------ */
extern volatile uint32_t uxSchedulerRunning;

BaseType_t xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
UBaseType_t uxTaskGetNumberOfTasks(void);

#ifdef __cplusplus
}
#endif

#include "task.h"
#include "queue.h"

#endif /* THUNDERS_FREERTOS_H */
