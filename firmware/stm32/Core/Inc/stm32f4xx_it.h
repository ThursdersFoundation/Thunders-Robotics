/**
 * @file    stm32f4xx_it.h
 * @brief   Interrupt handler declarations (strong implementations live in
 *          stm32f4xx_it.c; SysTick/PendSV/SVC come from the mini-RTOS port).
 */
#ifndef THUNDERS_STM32F4XX_IT_H
#define THUNDERS_STM32F4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

/* Peripherals used by the application */
void TIM5_IRQHandler(void);          /* ultrasonic input capture     */
void TIM6_DAC_IRQHandler(void);      /* HAL timebase                 */
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN1_SCE_IRQHandler(void);
void USART1_IRQHandler(void);        /* lidar errors                 */
void USART3_IRQHandler(void);        /* direct node link             */
void UART4_IRQHandler(void);         /* SBC link                     */
void DMA2_Stream0_IRQHandler(void);  /* ADC scan                     */
void EXTI15_10_IRQHandler(void);     /* user button                  */

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_STM32F4XX_IT_H */
