/**
 * @file    tim.h
 * @brief   Timer configuration:
 *   TIM2  - 20 kHz PWM for the 4 VNH5019 channels
 *   TIM1/3/4/8 - quadrature encoder counters
 *   TIM5  - 1 MHz input capture for the ultrasonic echoes
 *   TIM6  - 1 kHz HAL timebase (SysTick belongs to the RTOS)
 */
#ifndef THUNDERS_TIM_H
#define THUNDERS_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);
void MX_TIM8_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_TIM_H */
