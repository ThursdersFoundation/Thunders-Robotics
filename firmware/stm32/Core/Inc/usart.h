/**
 * @file    usart.h
 * @brief   UART links:
 *   USART1 256k  RPLidar A1 (DMA circular RX)
 *   USART2 115200  debug console
 *   USART3 115200  direct motor-node link (secondary path)
 *   UART4  921600  SBC link (DMA circular RX)
 */
#ifndef THUNDERS_USART_H
#define THUNDERS_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define UART_LIDAR_BAUD   256000U
#define UART_DEBUG_BAUD   115200U
#define UART_NODE_BAUD    115200U
#define UART_SBC_BAUD     921600U

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_UART4_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_USART_H */
