/**
 * @file    tim.c
 * @brief   Timer initialization for the drive system.
 *
 *   TIM2  PWM   PSC=0  ARR=4199  -> 84 MHz / 4200 = 20 kHz, duty via CCR
 *   TIM1  ENC   PE9/PE11, 4x mode, ARR=0xFFFF
 *   TIM3  ENC   PA6/PA7,  4x mode
 *   TIM4  ENC   PB6/PB7,  4x mode
 *   TIM8  ENC   PC6/PC7,  4x mode
 *   TIM5  IC    PSC=83 -> 1 MHz (1 us echo resolution)
 *   TIM6  BASE  PSC=83 ARR=999 -> 1 kHz HAL tick
 */
#include "tim.h"
#include "stm32f4xx_hal_tim.h"
#include "drv_motor.h"

static void TIM_EncoderCommon(TIM_HandleTypeDef *htim, TIM_TypeDef *inst)
{
    htim->Instance = inst;
    htim->Init.Prescaler = 0U;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFFU;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = { 0 };
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1.ICPolarity = TIM_ICPOLARITY_RISING;
    enc.IC1.ICSelection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1.ICPrescaler = TIM_ICPSC_DIV1;
    enc.IC1.ICFilter = 0x6U;              /* light glitch filter */
    enc.IC2 = enc.IC1;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    HAL_TIM_Encoder_Init(htim, &enc);
}

void MX_TIM1_Init(void) { TIM_EncoderCommon(&htim1, TIM1); }
void MX_TIM3_Init(void) { TIM_EncoderCommon(&htim3, TIM3); }
void MX_TIM4_Init(void) { TIM_EncoderCommon(&htim4, TIM4); }
void MX_TIM8_Init(void) { TIM_EncoderCommon(&htim8, TIM8); }

void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = DRV_MOTOR_PWM_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);

    /* Four channels, PWM mode 1, active high */
    for (uint32_t ch = TIM_CHANNEL_1; ch <= TIM_CHANNEL_4; ch++)
    {
        TIM_OC_InitTypeDef oc = { 0 };
        oc.OCMode = TIM_OCMODE_PWM1;
        oc.Pulse = 0U;
        oc.OCPolarity = TIM_OCPOLARITY_HIGH;
        oc.OCFastMode = 0U;
        oc.OCIdleState = TIM_OCIDLESTATE_RESET;
        HAL_TIM_PWM_ConfigChannel(&htim2, &oc, ch);
    }
}

void MX_TIM5_Init(void)
{
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 83U;            /* 84 MHz / 84 = 1 MHz */
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFU;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&htim5);

    for (uint32_t ch = TIM_CHANNEL_1; ch <= TIM_CHANNEL_4; ch++)
    {
        TIM_IC_InitTypeDef ic = { 0 };
        ic.ICPolarity = TIM_ICPOLARITY_RISING;   /* ISR toggles per edge */
        ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
        ic.ICPrescaler = TIM_ICPSC_DIV1;
        ic.ICFilter = 0x3U;                    /* ~0.5 us glitch filter */
        HAL_TIM_IC_ConfigChannel(&htim5, &ic, ch);
    }
}

void MX_TIM6_Init(void)
{
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83U;            /* 84 MHz / 84 = 1 MHz */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 999U;              /* 1 kHz update */
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);
}
