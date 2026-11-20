/**
 * @file    tasks.c
 * @brief   Mini-RTOS scheduler: task creation, ready lists, delays.
 *
 * Scheduling model: one FIFO ready list per priority. The running task is
 * outside the lists; PendSV re-queues it (round-robin within a priority)
 * when it is still READY, then dequeues the head of the highest non-empty
 * priority via vPortSelectNextTask().
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "core_cm4.h"
#include "string.h"

#define MAX_TASKS 8U   /* pxTaskTable size; covers the 6 application tasks */

/* ------------------------------------------------------------------ */
/* Scheduler state                                                     */
/* ------------------------------------------------------------------ */
TCB_t *pxTaskTable[MAX_TASKS];
volatile UBaseType_t uxCurrentNumberOfTasks = 0U;

/* One ready list per priority: head + tail, FIFO order */
static TCB_t *pxReadyHead[configMAX_PRIORITIES];
static TCB_t *pxReadyTail[configMAX_PRIORITIES];

static TCB_t xIdleTaskTCB;
static StackType_t uxIdleStack[configMINIMAL_STACK_SIZE];

/* ------------------------------------------------------------------ */
/* Ready list management                                               */
/* ------------------------------------------------------------------ */
void vPortAddToReadyList(TCB_t *pxTCB)
{
    UBaseType_t prio = pxTCB->uxPriority;
    pxTCB->pxNext = NULL;
    pxTCB->pxPrev = pxReadyTail[prio];
    if (pxReadyTail[prio] != NULL)
    {
        pxReadyTail[prio]->pxNext = pxTCB;
    }
    else
    {
        pxReadyHead[prio] = pxTCB;
    }
    pxReadyTail[prio] = pxTCB;
}

void vPortRemoveFromReadyList(TCB_t *pxTCB)
{
    UBaseType_t prio = pxTCB->uxPriority;
    if (pxTCB->pxPrev != NULL)
    {
        pxTCB->pxPrev->pxNext = pxTCB->pxNext;
    }
    else
    {
        pxReadyHead[prio] = pxTCB->pxNext;
    }
    if (pxTCB->pxNext != NULL)
    {
        pxTCB->pxNext->pxPrev = pxTCB->pxPrev;
    }
    else
    {
        pxReadyTail[prio] = pxTCB->pxPrev;
    }
    pxTCB->pxNext = NULL;
    pxTCB->pxPrev = NULL;
}

/**
 * @brief Pick the next task (called from PendSV with interrupts disabled).
 *
 * Re-queues the outgoing task when it is still READY, then dequeues the
 * head of the highest-priority non-empty ready list.
 */
void vPortSelectNextTask(void)
{
    TCB_t *pxOutgoing = pxCurrentTCB;

    if (pxOutgoing != NULL && pxOutgoing->ucState == TASK_READY)
    {
        vPortAddToReadyList(pxOutgoing);
    }

    TCB_t *pxNext = NULL;
    for (BaseType_t prio = (BaseType_t)configMAX_PRIORITIES - 1; prio >= 0; prio--)
    {
        if (pxReadyHead[prio] != NULL)
        {
            pxNext = pxReadyHead[prio];
            vPortRemoveFromReadyList(pxNext);
            break;
        }
    }

    /* Scheduler invariant: the idle task is always runnable */
    if (pxNext == NULL)
    {
        pxNext = &xIdleTaskTCB;
    }
    pxCurrentTCB = pxNext;
}

/* ------------------------------------------------------------------ */
/* Task creation                                                       */
/* ------------------------------------------------------------------ */
BaseType_t xTaskCreate(TaskFunction_t pxTaskCode, const char *const pcName,
                       uint16_t usStackDepth, void *pvParameters,
                       UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask)
{
    if (uxCurrentNumberOfTasks >= MAX_TASKS ||
        uxPriority >= configMAX_PRIORITIES ||
        pxTaskCode == NULL)
    {
        return pdFAIL;
    }

    TCB_t *pxNewTCB = (TCB_t *)pvPortMalloc(sizeof(TCB_t));
    if (pxNewTCB == NULL) { return pdFAIL; }

    StackType_t *pxStack = (StackType_t *)pvPortMalloc((size_t)usStackDepth * sizeof(StackType_t));
    if (pxStack == NULL)
    {
        vPortFree(pxNewTCB);
        return pdFAIL;
    }

    memset(pxNewTCB, 0, sizeof(TCB_t));
    pxNewTCB->pxTopOfStack = pxPortInitialiseStack(pxStack + usStackDepth, pxTaskCode, pvParameters);
    pxNewTCB->pxStackBase  = pxStack;
    pxNewTCB->ulStackWords = usStackDepth;
    pxNewTCB->uxPriority   = uxPriority;
    pxNewTCB->ucState      = TASK_READY;
    pxNewTCB->pcTaskName   = pcName;

    pxTaskTable[uxCurrentNumberOfTasks++] = pxNewTCB;
    vPortAddToReadyList(pxNewTCB);

    if (pxCreatedTask != NULL) { *pxCreatedTask = pxNewTCB; }
    traceTASK_CREATE(pxNewTCB);
    return pdPASS;
}

/* ------------------------------------------------------------------ */
/* Delays                                                              */
/* ------------------------------------------------------------------ */
void vTaskDelay(TickType_t xTicksToDelay)
{
    if (pxCurrentTCB == NULL) { return; }

    if (xTicksToDelay == 0U)
    {
        vPortYield();
        return;
    }

    taskENTER_CRITICAL();
    pxCurrentTCB->ucState    = TASK_DELAYED;
    pxCurrentTCB->xWakeTick  = xTickCount + xTicksToDelay;
    pxCurrentTCB->ucTimedWait = 1U;
    taskEXIT_CRITICAL();

    vPortYield();   /* PendSV: state != READY so the task is not re-queued */
}

BaseType_t vTaskDelayUntil(TickType_t *const pxPreviousWakeTime, TickType_t xTimeIncrement)
{
    BaseType_t xAlreadyDelayed = pdFALSE;
    TickType_t xTimeToWake;

    taskENTER_CRITICAL();
    xTimeToWake = *pxPreviousWakeTime + xTimeIncrement;

    /* Missed deadline (e.g. after a suspend): run immediately, resync */
    if ((int32_t)(xTimeToWake - xTickCount) <= 0)
    {
        *pxPreviousWakeTime = xTickCount;
        taskEXIT_CRITICAL();
        return xAlreadyDelayed;
    }

    *pxPreviousWakeTime = xTimeToWake;
    pxCurrentTCB->ucState    = TASK_DELAYED;
    pxCurrentTCB->xWakeTick  = xTimeToWake;
    pxCurrentTCB->ucTimedWait = 1U;
    taskEXIT_CRITICAL();

    vPortYield();
    return xAlreadyDelayed;
}

/* ------------------------------------------------------------------ */
/* Scheduler start / queries                                           */
/* ------------------------------------------------------------------ */
static void prvIdleTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        __WFI();   /* wait for the next tick/interrupt */
    }
}

void vTaskStartScheduler(void)
{
    /* Idle task uses static storage so the pool never runs dry */
    memset(&xIdleTaskTCB, 0, sizeof(xIdleTaskTCB));
    xIdleTaskTCB.pxTopOfStack = pxPortInitialiseStack(uxIdleStack + configMINIMAL_STACK_SIZE,
                                                      prvIdleTask, NULL);
    xIdleTaskTCB.pxStackBase  = uxIdleStack;
    xIdleTaskTCB.ulStackWords = configMINIMAL_STACK_SIZE;
    xIdleTaskTCB.uxPriority   = 0U;
    xIdleTaskTCB.ucState      = TASK_READY;
    xIdleTaskTCB.pcTaskName   = "IDLE";

    uxSchedulerRunning = 1U;

    /* SysTick + handler priorities (PendSV/SysTick lowest, SVC high) */
    vPortSetupTickTimer();

    vPortStartFirstTask();   /* SVC 0 -> first task */

    /* Unreachable */
    for (;;) { }
}

BaseType_t xTaskGetSchedulerState(void)
{
    return (uxSchedulerRunning != 0U) ? pdTRUE : pdFALSE;
}

TickType_t xTaskGetTickCount(void)
{
    return xTickCount;
}

UBaseType_t uxTaskGetNumberOfTasks(void)
{
    return uxCurrentNumberOfTasks;
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return pxCurrentTCB;
}
