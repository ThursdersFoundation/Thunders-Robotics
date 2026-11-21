/**
 * @file    can.h
 * @brief   CAN1: 500 kbit/s node backbone (PD0/PD1 + external transceiver).
 */
#ifndef THUNDERS_CAN_H
#define THUNDERS_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void);
void CAN_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_CAN_H */
