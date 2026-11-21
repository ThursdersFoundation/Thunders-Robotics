/**
 * @file    usart.c
 * @brief   UART configuration for lidar / debug / node / SBC links.
 */
#include "usart.h"
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;

static void Uart_Init(UART_HandleTypeDef *huart, USART_TypeDef *inst, uint32_t baud)
{
    huart->Instance = inst;
    huart->Init.BaudRate = baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(huart);
}

void MX_USART1_UART_Init(void)  { Uart_Init(&huart1, USART1, UART_LIDAR_BAUD);  }
void MX_USART2_UART_Init(void)  { Uart_Init(&huart2, USART2, UART_DEBUG_BAUD);  }
void MX_USART3_UART_Init(void)  { Uart_Init(&huart3, USART3, UART_NODE_BAUD);   }
void MX_UART4_Init(void)        { Uart_Init(&huart4, UART4,  UART_SBC_BAUD);    }
