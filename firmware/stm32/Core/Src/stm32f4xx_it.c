/**
 * @file    stm32f4xx_it.c
 * @brief   Interrupt service routines for the Thunders Robotic firmware.
 *
 * SysTick_Handler / PendSV_Handler / SVC_Handler are NOT here: they are
 * provided by the mini-RTOS port (Middlewares/FreeRTOS/Src/port.c) and the
 * weak aliases in startup_stm32f407xx.s are overridden.
 */
#include "main.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "safety_manager.h"
#include "drv_ultrasonic.h"
#include "drv_lidar.h"
#include "communication_manager.h"

/* ------------------------------------------------------------------ */
/* Core exceptions                                                     */
/* ------------------------------------------------------------------ */
void NMI_Handler(void)      { for (;;) { } }
void DebugMon_Handler(void) { }

void HardFault_Handler(void)
{
    SafetyManager_RaiseFault(FAULT_WATCHDOG, 0U);
    for (;;) { }
}

void MemManage_Handler(void) { HardFault_Handler(); }
void BusFault_Handler(void)  { HardFault_Handler(); }
void UsageFault_Handler(void) { HardFault_Handler(); }

/* ------------------------------------------------------------------ */
/* TIM6: HAL timebase @ 1 kHz                                          */
/* ------------------------------------------------------------------ */
void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) != 0U)
    {
        TIM6->SR = (uint32_t)~TIM_SR_UIF;
        HAL_IncTick();
    }
}

/* ------------------------------------------------------------------ */
/* TIM5: ultrasonic echo capture. The polarity toggles per edge so the
 * same channel measures rise and fall of one echo pulse.               */
/* ------------------------------------------------------------------ */
void TIM5_IRQHandler(void)
{
    uint32_t sr = TIM5->SR;

    for (uint32_t ch = 1U; ch <= 4U; ch++)
    {
        uint32_t flag = 1UL << ch;   /* CCxIF */
        if ((sr & flag) != 0U)
        {
            uint16_t captured = (uint16_t)TIM5->CCR[ch - 1U];
            TIM5->SR = (uint32_t)~flag;
            DrvUltrasonic_OnCapture((uint8_t)(ch - 1U), captured);

            /* Toggle edge polarity for the next phase of the pulse */
            uint32_t bit = 1UL << ((ch - 1U) * 4U + 1U);   /* CCxP */
            TIM5->CCER ^= bit;
        }
    }
}

/* ------------------------------------------------------------------ */
/* CAN1                                                                */
/* ------------------------------------------------------------------ */
void CAN1_TX_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan = &hcan1;
    uint32_t tsr = CAN1->TSR;

    if (tsr & CAN_TSR_RQCP0)
    {
        CAN1->TSR = CAN_TSR_RQCP0;
        HAL_CAN_TxMailbox0CompleteCallback(hcan);
    }
    if (tsr & CAN_TSR_RQCP1)
    {
        CAN1->TSR = CAN_TSR_RQCP1;
        HAL_CAN_TxMailbox1CompleteCallback(hcan);
    }
    if (tsr & CAN_TSR_RQCP2)
    {
        CAN1->TSR = CAN_TSR_RQCP2;
        HAL_CAN_TxMailbox2CompleteCallback(hcan);
    }
}

static void CAN1_DrainFifo(volatile uint32_t *rfr)
{
    while (((*rfr) & CAN_RF0R_FMP0_Mask) != 0U)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(&hcan1, 0U, &header, data) == HAL_OK)
        {
            CommunicationManager_OnCanFrame(header.StdId, data, (uint8_t)header.DLC);
        }
        *rfr = CAN_RF0R_RFOM0;
    }
}

void CAN1_RX0_IRQHandler(void)
{
    CAN1_DrainFifo(&CAN1->RF0R);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

void CAN1_RX1_IRQHandler(void)
{
    CAN1_DrainFifo(&CAN1->RF1R);
    HAL_CAN_RxFifo1MsgPendingCallback(&hcan1);
}

void CAN1_SCE_IRQHandler(void)
{
    uint32_t esr = CAN1->ESR;
    if (esr & CAN_ESR_LEC_Mask)
    {
        CAN1->ESR &= ~CAN_ESR_LEC_Mask;   /* clear LEC */
        hcan1.ErrorCode |= HAL_CAN_ERROR_EWG;
        HAL_CAN_ErrorCallback(&hcan1);
    }
}

/* ------------------------------------------------------------------ */
/* UART error handlers (data comes via DMA; only faults are handled)   */
/* ------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        (void)USART1->DR;                 /* clear the error sequence */
    }
}

void USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        (void)USART3->DR;
    }
}

void UART4_IRQHandler(void)
{
    uint32_t sr = UART4->SR;
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        (void)UART4->DR;
    }
}

/* ------------------------------------------------------------------ */
/* DMA2 Stream0: ADC scan complete (data read by the battery driver)   */
/* ------------------------------------------------------------------ */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF7) { /* not ours */ }
    if (DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;   /* circular mode: just clear */
    }
}

/* ------------------------------------------------------------------ */
/* User button (PC13): emergency stop request                          */
/* ------------------------------------------------------------------ */
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & USER_BTN_Pin) != 0U)
    {
        EXTI->PR = USER_BTN_Pin;
        SafetyManager_EStopButtonPressed();
    }
}
