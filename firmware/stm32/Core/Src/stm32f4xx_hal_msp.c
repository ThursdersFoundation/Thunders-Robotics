/**
 * @file    stm32f4xx_hal_msp.c
 * @brief   MSP init: peripheral clocks, alternate-function GPIO and NVIC
 *          priorities. Called by HAL_xxx_Init() for every peripheral.
 */
#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_can.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;

/* F4 AF9 constant (CAN1 on PD0/PD1) */
#ifndef GPIO_AF9_CAN1
#define GPIO_AF9_CAN1 9UL
#endif

/* ------------------------------------------------------------------ */
/* GPIO helper                                                         */
/* ------------------------------------------------------------------ */
static void Msp_AF(GPIO_TypeDef *port, uint32_t pin, uint32_t alternate, uint32_t speed)
{
    GPIO_InitTypeDef init = { 0 };
    init.Pin = pin;
    init.Mode = GPIO_MODE_AF;
    init.Pull = GPIO_NOPULL;
    init.Speed = speed;
    init.Alternate = alternate;
    HAL_GPIO_Init(port, &init);
}

static void Msp_Analog(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef init = { 0 };
    init.Pin = pin;
    init.Mode = GPIO_MODE_ANALOG;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &init);
}

/* ------------------------------------------------------------------ */
/* TIM: PWM (TIM2), encoders (TIM1/3/4/8), capture (TIM5), tick (TIM6) */
/* ------------------------------------------------------------------ */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        Msp_AF(GPIOA, GPIO_PIN_5,  GPIO_AF1_TIM2, GPIO_SPEED_HIGH);  /* CH1 PA5  */
        Msp_AF(GPIOB, GPIO_PIN_3,  GPIO_AF1_TIM2, GPIO_SPEED_HIGH);  /* CH2 PB3  */
        Msp_AF(GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2, GPIO_SPEED_HIGH);  /* CH3 PB10 */
        Msp_AF(GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, GPIO_SPEED_HIGH);  /* CH4 PB11 */
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TICK_INT_PRIORITY, 0U);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_TIM_MspPostInit(htim);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        Msp_AF(GPIOE, GPIO_PIN_9,  GPIO_AF0_TIM1, GPIO_SPEED_HIGH);  /* CH1 */
        Msp_AF(GPIOE, GPIO_PIN_11, GPIO_AF0_TIM1, GPIO_SPEED_HIGH);  /* CH2 */
    }
    else if (htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        Msp_AF(GPIOA, GPIO_PIN_6, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH1 */
        Msp_AF(GPIOA, GPIO_PIN_7, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH2 */
    }
    else if (htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        Msp_AF(GPIOB, GPIO_PIN_6, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH1 */
        Msp_AF(GPIOB, GPIO_PIN_7, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH2 */
    }
    else if (htim->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        Msp_AF(GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8, GPIO_SPEED_HIGH);   /* CH1 */
        Msp_AF(GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8, GPIO_SPEED_HIGH);   /* CH2 */
    }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        Msp_AF(GPIOA, GPIO_PIN_0, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH1 */
        Msp_AF(GPIOA, GPIO_PIN_1, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH2 */
        Msp_AF(GPIOA, GPIO_PIN_2, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH3 */
        Msp_AF(GPIOA, GPIO_PIN_3, GPIO_AF2_TIM345, GPIO_SPEED_HIGH); /* CH4 */
        HAL_NVIC_SetPriority(TIM5_IRQn, 6U, 0U);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
    }
}

/* ------------------------------------------------------------------ */
/* UART                                                                */
/* ------------------------------------------------------------------ */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        Msp_AF(GPIOA, GPIO_PIN_9,  GPIO_AF7_USART, GPIO_SPEED_VERY_HIGH); /* TX */
        Msp_AF(GPIOA, GPIO_PIN_10, GPIO_AF7_USART, GPIO_SPEED_VERY_HIGH); /* RX */
    }
    else if (huart->Instance == USART2)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        Msp_AF(GPIOD, GPIO_PIN_5, GPIO_AF7_USART, GPIO_SPEED_MEDIUM);     /* TX */
        Msp_AF(GPIOD, GPIO_PIN_6, GPIO_AF7_USART, GPIO_SPEED_MEDIUM);     /* RX */
    }
    else if (huart->Instance == USART3)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        Msp_AF(GPIOD, GPIO_PIN_8, GPIO_AF7_USART, GPIO_SPEED_MEDIUM);     /* TX */
        Msp_AF(GPIOD, GPIO_PIN_9, GPIO_AF7_USART, GPIO_SPEED_MEDIUM);     /* RX */
    }
    else if (huart->Instance == UART4)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        Msp_AF(GPIOC, GPIO_PIN_10, GPIO_AF7_USART, GPIO_SPEED_VERY_HIGH); /* TX */
        Msp_AF(GPIOC, GPIO_PIN_11, GPIO_AF7_USART, GPIO_SPEED_VERY_HIGH); /* RX */
    }
}

/* ------------------------------------------------------------------ */
/* I2C1 (PB8/PB9)                                                      */
/* ------------------------------------------------------------------ */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitTypeDef init = { 0 };
        init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        init.Mode = GPIO_MODE_AF;
        init.Pull = GPIO_PULLUP;
        init.Speed = GPIO_SPEED_HIGH;
        init.Alternate = GPIO_AF4_I2C;
        HAL_GPIO_Init(GPIOB, &init);
    }
}

/* ------------------------------------------------------------------ */
/* SPI2 (PB12-PB15)                                                    */
/* ------------------------------------------------------------------ */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        Msp_AF(GPIOB, GPIO_PIN_12, GPIO_AF5_SPI, GPIO_SPEED_HIGH);  /* NSS (soft) */
        Msp_AF(GPIOB, GPIO_PIN_13, GPIO_AF5_SPI, GPIO_SPEED_HIGH);  /* SCK  */
        Msp_AF(GPIOB, GPIO_PIN_14, GPIO_AF5_SPI, GPIO_SPEED_HIGH);  /* MISO */
        Msp_AF(GPIOB, GPIO_PIN_15, GPIO_AF5_SPI, GPIO_SPEED_HIGH);  /* MOSI */
    }
}

/* ------------------------------------------------------------------ */
/* ADC1 (PC0/PC1/PC2)                                                  */
/* ------------------------------------------------------------------ */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        Msp_Analog(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    }
}

/* ------------------------------------------------------------------ */
/* CAN1 (PD0/PD1)                                                      */
/* ------------------------------------------------------------------ */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        Msp_AF(GPIOD, GPIO_PIN_0, GPIO_AF9_CAN1, GPIO_SPEED_HIGH);  /* RX */
        Msp_AF(GPIOD, GPIO_PIN_1, GPIO_AF9_CAN1, GPIO_SPEED_HIGH);  /* TX */
    }
}
