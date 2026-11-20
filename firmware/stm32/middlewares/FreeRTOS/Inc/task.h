/**
 * @file    task.h
 * @brief   Task API (subset used by the application).
 */
#ifndef THUNDERS_TASK_H
#define THUNDERS_TASK_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Task states (internal; exposed for the TCB layout) */
#define TASK_READY    ((uint8_t)0)
#define TASK_DELAYED  ((uint8_t)1)
#define TASK_BLOCKED  ((uint8_t)2)

/**
 * @brief Task control block.
 *
 * pxTopOfStack MUST stay the first member: the PendSV handler stores/loads
 * the task stack pointer through the TCB pointer directly.
 */
typedef struct tskTCB
{
    volatile StackType_t *pxTopOfStack;
    struct tskTCB *pxNext;         /* ready-list link                   */
    struct tskTCB *pxPrev;         /* ready-list link                   */
    struct tskTCB *pxWaitNext;     /* queue waiter list link            */
    struct tskTCB *pxWaitPrev;
    void           *pvWaitObject;  /* queue the task is blocked on      */
    TickType_t      xWakeTick;     /* absolute wake-up tick             */
    UBaseType_t     uxPriority;
    uint8_t         ucState;
    uint8_t         ucTimedWait;   /* blocked with timeout -> tick can wake */
    int8_t          xWaitResult;   /* pdTRUE: object ready, pdFALSE: timeout */
    StackType_t    *pxStackBase;
    uint32_t        ulStackWords;
    const char     *pcTaskName;
} TCB_t;

typedef TCB_t *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

/**
 * @brief Create a task from the static kernel pool.
 * @param usStackDepth stack size in WORDS (like FreeRTOS).
 */
BaseType_t xTaskCreate(TaskFunction_t pxTaskCode, const char *const pcName,
                       uint16_t usStackDepth, void *pvParameters,
                       UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask);

void vTaskStartScheduler(void);
void vTaskDelay(TickType_t xTicksToDelay);
BaseType_t vTaskDelayUntil(TickType_t *const pxPreviousWakeTime, TickType_t xTimeIncrement);
void vTaskSuspendAll(void);      /* cooperative: keeps tasks running, stops ticks? simplified */
BaseType_t xTaskResumeAll(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);

/* Internal (port.c) */
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters);
void vPortTaskExit(void);
void vPortSetupTickTimer(void);
void vPortStartFirstTask(void);

void vPortSelectNextTask(void);
void vPortAddToReadyList(TCB_t *pxTCB);
void vPortRemoveFromReadyList(TCB_t *pxTCB);
extern TCB_t *volatile pxCurrentTCB;
extern volatile TickType_t xTickCount;
extern TCB_t *pxTaskTable[8];
extern volatile UBaseType_t uxCurrentNumberOfTasks;

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_TASK_H */
