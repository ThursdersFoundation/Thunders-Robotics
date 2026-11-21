/**
 * @file    mock_hal.h
 * @brief   Host-side mock of the hardware layer for unit tests.
 *
 * Provides a controllable HAL_GetTick, recording GPIO, capture UART and
 * fake peripheral handles so driver code links and runs on x86.
 */
#ifndef THUNDERS_MOCK_HAL_H
#define THUNDERS_MOCK_HAL_H

#include "stm32f407xx.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim6, htim8;
extern UART_HandleTypeDef huart1, huart2, huart3, huart4;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;

void Mock_SetTick(uint32_t ms);
void Mock_AdvanceTick(uint32_t ms);

typedef struct { uint32_t portId, pin, writes, lastState; } MockGpioStat;
void Mock_GpioReset(void);
MockGpioStat Mock_GpioStat(uint32_t portId, uint32_t pin);

extern uint8_t  mock_uart_tx[4][256];   /* USART1/2/3/UART4 TX capture */
extern volatile uint16_t mock_uart_tx_len[4];
void Mock_UartReset(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_MOCK_HAL_H */

/* Extras for host tests */
void Mock_SetTimerCount(TIM_HandleTypeDef *htim, uint16_t cnt);
uint16_t Mock_GetTimerCount(TIM_HandleTypeDef *htim);
int  Mock_WatchdogRefreshes(void);
void Mock_I2cSetWhoAmI(uint8_t id);
void Mock_I2cSetBurst14(const uint8_t *raw);
