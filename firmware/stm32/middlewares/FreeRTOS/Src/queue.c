/**
 * @file    queue.c
 * @brief   Mini-RTOS queues: copy ring buffer + blocking send/receive.
 *
 * Waiter model: each queue keeps doubly-linked lists of tasks blocked on
 * send (buffer full) / receive (buffer empty). Waking is done either by the
 * counterpart operation or by the SysTick timeout scan in port.c.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Waiter list helpers                                                 */
/* ------------------------------------------------------------------ */
void vQueueRemoveWaiter(TCB_t **ppHead, TCB_t *pxTCB)
{
    if (pxTCB->pxWaitPrev != NULL)
    {
        pxTCB->pxWaitPrev->pxWaitNext = pxTCB->pxWaitNext;
    }
    else if (*ppHead == pxTCB)
    {
        *ppHead = pxTCB->pxWaitNext;
    }
    if (pxTCB->pxWaitNext != NULL)
    {
        pxTCB->pxWaitNext->pxWaitPrev = pxTCB->pxWaitPrev;
    }
    pxTCB->pxWaitNext = NULL;
    pxTCB->pxWaitPrev = NULL;
    pxTCB->pvWaitObject = NULL;
}

/** Wake the first waiter with the given result and make it runnable. */
void vQueueWakeOne(TCB_t **ppHead, BaseType_t xResult)
{
    TCB_t *pxTask = *ppHead;
    if (pxTask == NULL) { return; }
    vQueueRemoveWaiter(ppHead, pxTask);
    pxTask->xWaitResult = (int8_t)xResult;
    pxTask->ucState = TASK_READY;
    vPortAddToReadyList(pxTask);
}

static void vQueueWakeOnePendIfHigher(TCB_t **ppHead)
{
    TCB_t *pxHead = *ppHead;
    if (pxHead == NULL) { return; }
    BaseType_t xHigher = (pxCurrentTCB != NULL && pxHead->uxPriority > pxCurrentTCB->uxPriority);
    vQueueWakeOne(ppHead, pdTRUE);
    if (xHigher)
    {
        vPortYield();
    }
}

/* ------------------------------------------------------------------ */
/* Queue lifecycle                                                     */
/* ------------------------------------------------------------------ */
QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize)
{
    if (uxQueueLength == 0U || uxItemSize == 0U) { return NULL; }

    Queue_t *pxQueue = (Queue_t *)pvPortMalloc(sizeof(Queue_t));
    if (pxQueue == NULL) { return NULL; }

    pxQueue->pucBuffer = (uint8_t *)pvPortMalloc((size_t)uxQueueLength * uxItemSize);
    if (pxQueue->pucBuffer == NULL)
    {
        vPortFree(pxQueue);
        return NULL;
    }
    pxQueue->uxLength   = uxQueueLength;
    pxQueue->uxItemSize = uxItemSize;
    pxQueue->uxCount    = 0U;
    pxQueue->uxHead     = 0U;
    pxQueue->uxTail     = 0U;
    pxQueue->pxRecvWaiters = NULL;
    pxQueue->pxSendWaiters = NULL;
    return pxQueue;
}

void vQueueDelete(QueueHandle_t xQueue)
{
    if (xQueue == NULL) { return; }
    vPortFree(xQueue->pucBuffer);
    vPortFree(xQueue);
}

UBaseType_t uxQueueMessagesWaiting(QueueHandle_t xQueue)
{
    return xQueue ? xQueue->uxCount : 0U;
}

/* ------------------------------------------------------------------ */
/* Buffer primitives (call with interrupts disabled)                   */
/* ------------------------------------------------------------------ */
static BaseType_t prvCopyToQueue(Queue_t *pxQueue, const void *pvItem, BaseType_t xToFront)
{
    if (pxQueue->uxCount >= pxQueue->uxLength) { return pdFALSE; }

    uint8_t *slot;
    if (xToFront && pxQueue->uxCount > 0U)
    {
        /* Move tail back by one (ring-safe via modular arithmetic) */
        pxQueue->uxTail = (pxQueue->uxTail + pxQueue->uxLength - 1U) % pxQueue->uxLength;
        slot = &pxQueue->pucBuffer[pxQueue->uxTail * pxQueue->uxItemSize];
    }
    else
    {
        slot = &pxQueue->pucBuffer[pxQueue->uxHead * pxQueue->uxItemSize];
        pxQueue->uxHead = (pxQueue->uxHead + 1U) % pxQueue->uxLength;
    }
    memcpy(slot, pvItem, pxQueue->uxItemSize);
    pxQueue->uxCount++;
    return pdTRUE;
}

static BaseType_t prvCopyFromQueue(Queue_t *pxQueue, void *pvBuffer, BaseType_t xPeek)
{
    if (pxQueue->uxCount == 0U) { return pdFALSE; }

    memcpy(pvBuffer, &pxQueue->pucBuffer[pxQueue->uxTail * pxQueue->uxItemSize], pxQueue->uxItemSize);
    if (!xPeek)
    {
        pxQueue->uxTail = (pxQueue->uxTail + 1U) % pxQueue->uxLength;
        pxQueue->uxCount--;
    }
    return pdTRUE;
}

/** Block the current task on a waiter list with an optional timeout. */
static void prvBlockCurrent(Queue_t *pxQueue, TCB_t **ppWaiterList, TickType_t xTicksToWait)
{
    TCB_t *pxTask = pxCurrentTCB;

    pxTask->ucState      = TASK_BLOCKED;
    pxTask->pvWaitObject = pxQueue;
    pxTask->xWaitResult  = pdFALSE;
    pxTask->ucTimedWait  = (xTicksToWait != portMAX_DELAY) ? 1U : 0U;
    pxTask->xWakeTick    = xTickCount + xTicksToWait;

    /* Append to the doubly-linked waiter list */
    pxTask->pxWaitNext = NULL;
    pxTask->pxWaitPrev = *ppWaiterList;
    if (*ppWaiterList != NULL)
    {
        (*ppWaiterList)->pxWaitNext = pxTask;
    }
    else
    {
        *ppWaiterList = pxTask;
    }
    /* The head pointer passed in lives inside the queue struct */
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
static BaseType_t prvQueueSendGeneric(QueueHandle_t xQueue, const void *pvItemToQueue,
                                      TickType_t xTicksToWait, BaseType_t xToFront)
{
    if (xQueue == NULL || pvItemToQueue == NULL) { return pdFAIL; }

    for (;;)
    {
        taskENTER_CRITICAL();
        if (prvCopyToQueue(xQueue, pvItemToQueue, xToFront) == pdTRUE)
        {
            vQueueWakeOnePendIfHigher(&xQueue->pxRecvWaiters);
            taskEXIT_CRITICAL();
            return pdPASS;
        }
        if (xTicksToWait == 0U || pxCurrentTCB == NULL)
        {
            taskEXIT_CRITICAL();
            return errQUEUE_FULL;
        }
        prvBlockCurrent(xQueue, &xQueue->pxSendWaiters, xTicksToWait);
        taskEXIT_CRITICAL();

        vPortYield();   /* switched out; woken with space or by timeout */

        if (pxCurrentTCB->xWaitResult == pdFALSE)
        {
            return errQUEUE_FULL;   /* timeout */
        }
        /* Got space (or a race): retry the copy */
    }
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait)
{
    return prvQueueSendGeneric(xQueue, pvItemToQueue, xTicksToWait, pdFALSE);
}

BaseType_t xQueueSendToFront(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait)
{
    return prvQueueSendGeneric(xQueue, pvItemToQueue, xTicksToWait, pdTRUE);
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait)
{
    if (xQueue == NULL || pvBuffer == NULL) { return pdFAIL; }

    for (;;)
    {
        taskENTER_CRITICAL();
        if (prvCopyFromQueue(xQueue, pvBuffer, pdFALSE) == pdTRUE)
        {
            vQueueWakeOnePendIfHigher(&xQueue->pxSendWaiters);
            taskEXIT_CRITICAL();
            return pdPASS;
        }
        if (xTicksToWait == 0U || pxCurrentTCB == NULL)
        {
            taskEXIT_CRITICAL();
            return errQUEUE_EMPTY;
        }
        prvBlockCurrent(xQueue, &xQueue->pxRecvWaiters, xTicksToWait);
        taskEXIT_CRITICAL();

        vPortYield();

        if (pxCurrentTCB->xWaitResult == pdFALSE)
        {
            return errQUEUE_EMPTY;   /* timeout */
        }
    }
}

BaseType_t xQueuePeek(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait)
{
    if (xQueue == NULL || pvBuffer == NULL) { return pdFAIL; }

    taskENTER_CRITICAL();
    BaseType_t ok = prvCopyFromQueue(xQueue, pvBuffer, pdTRUE);
    taskEXIT_CRITICAL();
    if (ok) { return pdPASS; }
    return (xTicksToWait == 0U) ? errQUEUE_EMPTY : pdFAIL; /* peek is non-blocking here */
}

BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (xQueue == NULL || pvItemToQueue == NULL) { return pdFAIL; }

    BaseType_t xWoken = pdFALSE;
    taskENTER_CRITICAL();
    if (prvCopyToQueue(xQueue, pvItemToQueue, pdFALSE) == pdTRUE)
    {
        TCB_t *pxHead = xQueue->pxRecvWaiters;
        if (pxHead != NULL && pxCurrentTCB != NULL &&
            pxHead->uxPriority > pxCurrentTCB->uxPriority)
        {
            xWoken = pdTRUE;
        }
        vQueueWakeOne(&xQueue->pxRecvWaiters, pdTRUE);
    }
    taskEXIT_CRITICAL();
    if (pxHigherPriorityTaskWoken != NULL) { *pxHigherPriorityTaskWoken = xWoken; }
    return pdPASS;
}

BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *pvBuffer, BaseType_t *pxHigherPriorityTaskWoken)
{
    if (xQueue == NULL || pvBuffer == NULL) { return pdFAIL; }

    BaseType_t xWoken = pdFALSE;
    BaseType_t xGot = pdFALSE;
    taskENTER_CRITICAL();
    if (prvCopyFromQueue(xQueue, pvBuffer, pdFALSE) == pdTRUE)
    {
        xGot = pdTRUE;
        TCB_t *pxHead = xQueue->pxSendWaiters;
        if (pxHead != NULL && pxCurrentTCB != NULL &&
            pxHead->uxPriority > pxCurrentTCB->uxPriority)
        {
            xWoken = pdTRUE;
        }
        vQueueWakeOne(&xQueue->pxSendWaiters, pdTRUE);
    }
    taskEXIT_CRITICAL();
    if (pxHigherPriorityTaskWoken != NULL) { *pxHigherPriorityTaskWoken = xWoken; }
    return xGot ? pdPASS : errQUEUE_EMPTY;
}
