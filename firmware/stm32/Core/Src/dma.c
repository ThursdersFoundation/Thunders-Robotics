/**
 * @file    dma.c
 * @brief   DMA clock + NVIC enable. Stream configuration happens inside the
 *          owning peripherals (ADC1 -> DMA2 S0, USART1 RX -> DMA2 S2,
 *          UART4 RX -> DMA2 S1, USART3 RX -> DMA1 S1).
 */
#include "dma.h"

void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Stream interrupt priorities: above the RTOS-safe threshold set used
     * by this kernel (PRIMASK model), below the control-loop timing IRQs */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 7U, 0U);   /* ADC1 */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 7U, 0U);   /* lidar RX   */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6U, 0U);   /* SBC RX     */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 7U, 0U);   /* motor link */

    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}
