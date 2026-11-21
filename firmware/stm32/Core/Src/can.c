/**
 * @file    can.c
 * @brief   CAN1 @ 500 kbit/s: 42 MHz APB1 / prescaler 7 = 6 MHz tq clock,
 *          12 tq/bit (1 sync + 9 BS1 + 2 BS2), sample point 83 %.
 *          Filter 0 accepts everything into FIFO0 (node filtering is done
 *          in software by the communication manager).
 */
#include "can.h"
#include "stm32f4xx_hal_can.h"

CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 7U;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = 1U;
    hcan1.Init.TimeSeg1 = 9U;    /* BS1: 9 tq  */
    hcan1.Init.TimeSeg2 = 2U;    /* BS2: 2 tq  */
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutomaticWakeUp = ENABLE;
    hcan1.Init.NoAutoRetransmission = DISABLE;
    hcan1.Init.RxFifoLockMode = DISABLE;
    hcan1.Init.TxFlushMode = DISABLE;
    HAL_CAN_Init(&hcan1);

    CAN_FilterTypeDef filter = { 0 };
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;   /* mask 0 = accept all */
    filter.FilterMaskIdLow = 0x0000U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

void CAN_Start(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 6U, 0U);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}
