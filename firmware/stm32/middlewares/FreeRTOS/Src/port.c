/**
 * @file    port.c
 * @brief   Cortex-M4 port of the mini-RTOS: context switch, SysTick,
 *          critical sections and the static kernel heap.
 *
 * Context model:
 *  - Threads run on PSP with 8-byte aligned exception frames, FPU disabled
 *    (soft-float build keeps the switch minimal and deterministic).
 *  - PendSV performs the register save/restore; the scheduling decision
 *    (which TCB is next) lives in tasks.c::vPortSelectNextTask.
 *  - SysTick (1 kHz) drives the tick count, wakes delayed/blocked tasks and
 *    pends PendSV when a higher-priority (or equal-priority round-robin)
 *    task becomes runnable.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

/* ------------------------------------------------------------------ */
/* Kernel heap (static bump allocator)                                 */
/* ------------------------------------------------------------------ */
static uint8_t ucKernelHeap[configTOTAL_HEAP_SIZE] __attribute__((aligned(8)));
static size_t uxHeapUsed = 0U;

void *pvPortMalloc(size_t xSize)
{
    /* 8-byte align */
    xSize = (xSize + 7U) & ~(size_t)7U;
    if (uxHeapUsed + xSize > sizeof(ucKernelHeap))
    {
        return NULL;   /* pool exhausted */
    }
    void *pv = &ucKernelHeap[uxHeapUsed];
    uxHeapUsed += xSize;
    return pv;
}

void vPortFree(void *pv)
{
    /* Tasks/queues in this firmware live for the lifetime of the system */
    (void)pv;
}

/* ------------------------------------------------------------------ */
/* Critical sections                                                   */
/* ------------------------------------------------------------------ */
static volatile UBaseType_t uxCriticalNesting = 0U;

void vPortEnterCritical(void)
{
    __disable_irq();
    uxCriticalNesting++;
    __DSB();
}

void vPortExitCritical(void)
{
    if (uxCriticalNesting > 0U)
    {
        uxCriticalNesting--;
        if (uxCriticalNesting == 0U)
        {
            __enable_irq();
        }
    }
}

volatile uint8_t s_hostYieldPending = 0U;   /* host build only (unused on ARM) */
uint8_t vPortTakeHostYield(void) { uint8_t v = s_hostYieldPending; s_hostYieldPending = 0U; return v; }

void vPortYield(void)
{
#if defined(__arm__) || defined(__thumb__)
    SCB->ICSR = (1UL << SCB_ICSR_PENDSVSET_Pos);
    __DSB();
    __ISB();
#else
    /* host build: record the request so unit tests can inspect it */
    s_hostYieldPending = 1U;
#endif
}

/* ------------------------------------------------------------------ */
/* Scheduler internals shared with tasks.c                             */
/* ------------------------------------------------------------------ */
TCB_t *volatile pxCurrentTCB = NULL;
volatile TickType_t xTickCount = 0U;
volatile uint32_t uxSchedulerRunning = 0U;

/* Defined in tasks.c / provided to the startup context */
void vPortTaskExit(void);
void vQueueRemoveWaiter(TCB_t **ppHead, TCB_t *pxTCB);

/**
 * @brief Initialize the saved context of a new task (see xTaskCreate).
 */
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters)
{
    /* Frame layout (top of allocated stack, descending):
     * [r4 r5 r6 r7 r8 r9 r10 r11][r0 r1 r2 r3 r12 lr pc xpsr]          */
    pxTopOfStack -= 16U;
    pxTopOfStack[0]  = 0x04040404UL;                 /* r4  */
    pxTopOfStack[1]  = 0x05050505UL;                 /* r5  */
    pxTopOfStack[2]  = 0x06060606UL;                 /* r6  */
    pxTopOfStack[3]  = 0x07070707UL;                 /* r7  */
    pxTopOfStack[4]  = 0x08080808UL;                 /* r8  */
    pxTopOfStack[5]  = 0x09090909UL;                 /* r9  */
    pxTopOfStack[6]  = 0x10101010UL;                 /* r10 */
    pxTopOfStack[7]  = 0x11111111UL;                 /* r11 */
    pxTopOfStack[8]  = (StackType_t)pvParameters;    /* r0  */
    pxTopOfStack[9]  = 0x02020202UL;                 /* r1  */
    pxTopOfStack[10] = 0x03030303UL;                 /* r2  */
    pxTopOfStack[11] = 0x04040405UL;                 /* r3  */
    pxTopOfStack[12] = 0x12121212UL;                 /* r12 */
    pxTopOfStack[13] = (StackType_t)vPortTaskExit;   /* lr  */
    pxTopOfStack[14] = (StackType_t)pxCode;          /* pc  */
    pxTopOfStack[15] = 0x01000000UL;                 /* xPSR: thumb bit */
    return pxTopOfStack;
}

/** Tasks must never return; trap here if one does. */
void vPortTaskExit(void)
{
    __disable_irq();
    for (;;) { }
}

#if defined(__arm__) || defined(__thumb__)

/**
 * @brief PendSV: switch context between PSP-based tasks.
 *
 * The outgoing task is re-queued by vPortSelectNextTask() only when it is
 * still READY (a task that blocked/delayed before the switch is not).
 */
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "   cpsid i                       \n"
        "   mrs r0, psp                   \n"
        "   stmdb r0!, {r4-r11}           \n"
        "   ldr r1, =pxCurrentTCB         \n"
        "   ldr r2, [r1]                  \n"
        "   str r0, [r2]                  \n"   /* save SP into TCB[0] */
        "   push {r3, lr}                 \n"
        "   bl vPortSelectNextTask        \n"
        "   ldr r1, =pxCurrentTCB         \n"
        "   ldr r0, [r1]                  \n"
        "   ldr r0, [r0]                  \n"   /* new SP              */
        "   ldmia r0!, {r4-r11}           \n"
        "   msr psp, r0                   \n"
        "   pop {r3, lr}                  \n"
        "   cpsie i                       \n"
        "   bx lr                         \n"
    );
}

/**
 * @brief SVC #0: start the first task (called from vTaskStartScheduler).
 */
__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "   cpsid i                       \n"
        "   ldr r1, =pxCurrentTCB         \n"
        "   ldr r0, [r1]                  \n"
        "   ldr r0, [r0]                  \n"
        "   ldmia r0!, {r4-r11}           \n"
        "   msr psp, r0                   \n"
        "   ldr lr, =0xFFFFFFFD           \n"   /* return to PSP, thread mode */
        "   cpsie i                       \n"
        "   bx lr                         \n"
    );
}

/**
 * @brief SysTick: kernel tick source at configTICK_RATE_HZ.
 */
void SysTick_Handler(void)
{
    BaseType_t xSwitchRequired = pdFALSE;

    taskENTER_CRITICAL();
    xTickCount++;

    /* Wake expired delays / blocked-with-timeout tasks */
    for (UBaseType_t i = 0U; i < uxCurrentNumberOfTasks; i++)
    {
        TCB_t *pxTask = pxTaskTable[i];
        if (pxTask == NULL ||
            (pxTask->ucState != TASK_DELAYED && !(pxTask->ucState == TASK_BLOCKED && pxTask->ucTimedWait)))
        {
            continue;
        }
        if ((int32_t)(pxTask->xWakeTick - xTickCount) <= 0)
        {
            if (pxTask->ucState == TASK_BLOCKED)
            {
                vQueueRemoveWaiter((TCB_t **)&pxTask->pvWaitObject, pxTask);
                pxTask->xWaitResult = pdFALSE;      /* timed out */
            }
            pxTask->ucState = TASK_READY;
            vPortAddToReadyList(pxTask);

            if (pxCurrentTCB != NULL)
            {
                if (pxTask->uxPriority > pxCurrentTCB->uxPriority)
                {
                    xSwitchRequired = pdTRUE;
                }
                else if (pxTask->uxPriority == pxCurrentTCB->uxPriority &&
                         configUSE_TIME_SLICING)
                {
                    xSwitchRequired = pdTRUE;       /* round-robin slice */
                }
            }
        }
    }
    taskEXIT_CRITICAL();

    if (xSwitchRequired)
    {
        vPortYield();
    }
}

#else /* host build: neutral stubs so the middleware compiles for unit tests */

void PendSV_Handler(void) {}
void SVC_Handler(void) {}
void SysTick_Handler(void) {}

#endif /* __arm__ */

/**
 * @brief Configure SysTick as the kernel tick (called with interrupts off).
 */
void vPortSetupTickTimer(void)
{
    SYSTICK->LOAD = (SystemCoreClock / (uint32_t)configTICK_RATE_HZ) - 1UL;
    SYSTICK->VAL  = 0UL;
    SYSTICK->CTRL = (1UL << SYSTICK_CTRL_CLKSOURCE_Pos) |
                    (1UL << SYSTICK_CTRL_TICKINT_Pos)   |
                    (1UL << SYSTICK_CTRL_ENABLE_Pos);

    /* Handler priorities: kernel handlers lowest, SVC high for start */
    SCB_SHPR_PENDSV  = 0xFFU;
    SCB_SHPR_SYSTICK = 0xFFU;
    SCB_SHPR_SVC     = 0x00U;
}

void vPortStartFirstTask(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile(
        "   cpsie i           \n"
        "   svc 0             \n"
        "   nop               \n"
    );
#else
    /* host build: no context switch to perform */
#endif
}
