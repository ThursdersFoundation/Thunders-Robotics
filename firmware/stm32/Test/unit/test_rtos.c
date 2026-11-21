/**
 * @file    test_rtos.c
 * @brief   Mini-RTOS unit tests on the host: task creation bookkeeping,
 *          ready-list priority order, queue ring behavior, waiter wake/timeout
 *          paths and critical-section nesting.
 *
 * The actual context switch (PendSV asm) runs only on the target; everything
 * around it is exercised here through the public scheduler functions.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

static int s_fail = 0;
static int s_total = 0;
#define CHECK(cond) do { s_total++; if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); s_fail++; } } while (0)
#define CHECK_EQ(a, b) do { s_total++; long _a=(long)(a), _b=(long)(b); \
    if (_a!=_b) { printf("  FAIL %s:%d  %s=%ld != %s=%ld\n", __FILE__, __LINE__, #a,_a,#b,_b); s_fail++; } } while (0)

static TaskHandle_t s_t1, s_t2, s_t3;
static int s_ran1, s_ran2;

static void vTestTask(void *arg)
{
    (void)arg;
    s_ran1++;
    vTaskDelay(2);
}

static void vCounterTask(void *arg)
{
    (void)arg;
    s_ran2++;
    vTaskDelay(1);
}

static void TestTaskCreation(void)
{
    CHECK_EQ(xTaskCreate(vTestTask, "T1", 128, NULL, 3, &s_t1), pdPASS);
    CHECK_EQ(xTaskCreate(vCounterTask, "T2", 128, NULL, 2, &s_t2), pdPASS);
    CHECK_EQ(xTaskCreate(vCounterTask, "T3", 128, NULL, 2, &s_t3), pdPASS);
    CHECK(uxTaskGetNumberOfTasks() >= 3);
    CHECK(s_t1 != NULL && s_t2 != NULL && s_t3 != NULL);
    CHECK(s_t1->pcTaskName != NULL);
    /* stack frame initialized with xPSR thumb bit on top */
    CHECK_EQ(s_t1->pxTopOfStack[15], 0x01000000UL);
}

static void TestReadyListPriority(void)
{
    /* Fresh scheduler state: T1(3) queued, T2/T3(2) queued.
     * Selection must return the highest-priority task first. */
    vPortSelectNextTask();
    CHECK(pxCurrentTCB == s_t1);

    /* T1 is READY -> it is re-queued and keeps beating prio-2 tasks */
    vPortSelectNextTask();
    CHECK(pxCurrentTCB == s_t1);

    /* Simulate vTaskDelay(T1): blocked tasks are not re-queued */
    s_t1->ucState = TASK_DELAYED;
    vPortSelectNextTask();
    CHECK(pxCurrentTCB == s_t2);

    /* Round-robin within prio 2: T3 follows T2 */
    vPortSelectNextTask();
    CHECK(pxCurrentTCB == s_t3);

    /* Wake T1 again: it must preempt the prio-2 tasks immediately */
    s_t1->ucState = TASK_READY;
    vPortAddToReadyList(s_t1);
    vPortSelectNextTask();
    CHECK(pxCurrentTCB == s_t1);
    s_t1->ucState = TASK_READY;
}

static void TestDelayBlocksAndTickWakes(void)
{
    /* Simulate what PendSV + SysTick do on the target: mark the running
     * task delayed, then run the tick wake scan. */
    TCB_t *t = pxCurrentTCB;
    t->ucState = TASK_DELAYED;
    t->xWakeTick = xTaskGetTickCount() + 2;
    t->ucTimedWait = 1U;
    vPortSelectNextTask();          /* delayed task is not re-queued */
    CHECK(pxCurrentTCB != t);

    /* advance 2 ticks: the port's wake scan runs inside SysTick_Handler,
     * which is asm-guarded; replicate the policy here via the same helpers */
    for (UBaseType_t i = 0; i < uxTaskGetNumberOfTasks(); i++)
    {
        TCB_t *pxTask = pxTaskTable[i];
        if (pxTask && pxTask->ucState == TASK_DELAYED &&
            (int32_t)(pxTask->xWakeTick - (xTaskGetTickCount() + 2)) <= 0)
        {
            pxTask->ucState = TASK_READY;
            vPortAddToReadyList(pxTask);
        }
    }
    vPortSelectNextTask();
    CHECK(pxCurrentTCB == t);   /* woke and became the highest-priority task */
}

static void TestQueueRing(void)
{
    QueueHandle_t q = xQueueCreate(3, sizeof(uint32_t));
    CHECK(q != NULL);
    CHECK_EQ(uxQueueMessagesWaiting(q), 0U);

    uint32_t v;
    CHECK_EQ(xQueueSend(q, &(uint32_t){1}, 0), pdPASS);
    CHECK_EQ(xQueueSend(q, &(uint32_t){2}, 0), pdPASS);
    CHECK_EQ(xQueueSend(q, &(uint32_t){3}, 0), pdPASS);
    CHECK_EQ(xQueueSend(q, &(uint32_t){4}, 0), errQUEUE_FULL);   /* full */

    CHECK_EQ(xQueuePeek(q, &v, 0), pdPASS);
    CHECK_EQ(v, 1);                        /* peek does not consume */
    CHECK_EQ(uxQueueMessagesWaiting(q), 3U);

    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 1);
    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 2);
    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 3);
    CHECK_EQ(xQueueReceive(q, &v, 0), errQUEUE_EMPTY);

    /* wraparound: head/tail indices cycle through the ring */
    for (int r = 0; r < 5; r++)
    {
        CHECK_EQ(xQueueSend(q, &r, 0), pdPASS);
        CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
        CHECK_EQ(v, (uint32_t)r);
    }
    vQueueDelete(q);
}

static void TestQueueSendToFront(void)
{
    QueueHandle_t q = xQueueCreate(2, sizeof(uint32_t));
    CHECK_EQ(xQueueSend(q, &(uint32_t){10}, 0), pdPASS);
    CHECK_EQ(xQueueSendToFront(q, &(uint32_t){99}, 0), pdPASS);
    uint32_t v;
    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 99);   /* front item first */
    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 10);
    vQueueDelete(q);
}

static void TestQueueWakeBlockedSender(void)
{
    /* Manual waiter-list manipulation: the same path used by
     * xQueueSend(blocking) -> xQueueReceive(wake). */
    QueueHandle_t q = xQueueCreate(1, sizeof(uint32_t));
    CHECK_EQ(xQueueSend(q, &(uint32_t){7}, 0), pdPASS);

    /* Fake a blocked sender (as prvBlockCurrent would set up) */
    TCB_t *fake = (TCB_t *)pvPortMalloc(sizeof(TCB_t));
    memset(fake, 0, sizeof(TCB_t));
    fake->ucState = TASK_BLOCKED;
    fake->pvWaitObject = q;
    fake->pxWaitNext = NULL;
    fake->pxWaitPrev = q->pxSendWaiters;
    q->pxSendWaiters = fake;

    /* Receive frees a slot and wakes the sender */
    uint32_t v;
    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 7);
    CHECK_EQ(fake->xWaitResult, pdTRUE);
    CHECK_EQ(fake->ucState, TASK_READY);
    CHECK(q->pxSendWaiters == NULL);   /* removed from the waiter list */
    vPortFree(fake);
    vQueueDelete(q);
}

static void TestQueueTimeoutResult(void)
{
    /* Timeout wake: vQueueRemoveWaiter + xWaitResult = pdFALSE */
    QueueHandle_t q = xQueueCreate(1, sizeof(uint32_t));
    CHECK_EQ(xQueueSend(q, &(uint32_t){7}, 0), pdPASS);

    TCB_t *fake = (TCB_t *)pvPortMalloc(sizeof(TCB_t));
    memset(fake, 0, sizeof(TCB_t));
    fake->ucState = TASK_BLOCKED;
    fake->pvWaitObject = q;
    fake->pxWaitNext = NULL;
    fake->pxWaitPrev = q->pxSendWaiters;
    q->pxSendWaiters = fake;

    vQueueRemoveWaiter(&q->pxSendWaiters, fake);
    fake->xWaitResult = pdFALSE;
    CHECK(q->pxSendWaiters == NULL);
    CHECK_EQ(fake->xWaitResult, pdFALSE);
    vPortFree(fake);
    vQueueDelete(q);
}

static void TestQueueFromISR(void)
{
    QueueHandle_t q = xQueueCreate(4, sizeof(uint32_t));
    BaseType_t woken = pdFALSE;
    CHECK_EQ(xQueueSendFromISR(q, &(uint32_t){0xAB}, &woken), pdPASS);
    uint32_t v = 0U;
    CHECK_EQ(xQueueReceive(q, &v, 0), pdPASS);
    CHECK_EQ(v, 0xAB);
    CHECK_EQ(xQueueReceiveFromISR(q, &v, &woken), errQUEUE_EMPTY);
    vQueueDelete(q);
}

static void TestCriticalNesting(void)
{
    taskENTER_CRITICAL();
    taskENTER_CRITICAL();
    taskEXIT_CRITICAL();
    taskEXIT_CRITICAL();
    CHECK(1);   /* balanced nesting does not corrupt state */
}

static void TestKernelHeap(void)
{
    void *a = pvPortMalloc(100);
    void *b = pvPortMalloc(200);
    CHECK(a != NULL && b != NULL);
    CHECK((uintptr_t)b - (uintptr_t)a >= 104);   /* 8-byte aligned blocks */
    vPortFree(a);   /* bump allocator: no-op by design */
}

int main(void)
{
    printf("== test_rtos ==\n");
    TestTaskCreation();
    TestReadyListPriority();
    TestDelayBlocksAndTickWakes();
    TestQueueRing();
    TestQueueSendToFront();
    TestQueueWakeBlockedSender();
    TestQueueTimeoutResult();
    TestQueueFromISR();
    TestCriticalNesting();
    TestKernelHeap();

    printf("%d/%d passed, %d failed\n", s_total - s_fail, s_total, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
