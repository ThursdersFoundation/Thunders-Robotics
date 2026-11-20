/**
 * @file    queue.h
 * @brief   Queue API (copy-in/copy-out ring buffer with blocking send/receive).
 */
#ifndef THUNDERS_QUEUE_H
#define THUNDERS_QUEUE_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct QueueDefinition
{
    uint8_t    *pucBuffer;
    UBaseType_t uxLength;     /* capacity in items  */
    UBaseType_t uxItemSize;   /* bytes per item     */
    volatile UBaseType_t uxCount;
    UBaseType_t uxHead;       /* next write index   */
    UBaseType_t uxTail;       /* next read index    */
    TCB_t      *pxRecvWaiters;
    TCB_t      *pxSendWaiters;
} Queue_t;

typedef Queue_t *QueueHandle_t;

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);
BaseType_t xQueueSendToFront(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait);
BaseType_t xQueuePeek(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait);
BaseType_t xQueueSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *pvBuffer, BaseType_t *pxHigherPriorityTaskWoken);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t xQueue);
void vQueueDelete(QueueHandle_t xQueue);

/* Internal helpers shared with tasks.c tick processing */
void vQueueRemoveWaiter(TCB_t **ppHead, TCB_t *pxTCB);
void vQueueWakeOne(TCB_t **ppHead, BaseType_t xResult);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_QUEUE_H */
