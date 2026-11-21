/**
 * @file    stm32f4xx_hal_conf.h
 * @brief   HAL configuration for the Thunders Robotic firmware.
 *
 * The project ships a compact, self-consistent HAL subset; this header keeps
 * the standard switch layout so the official ST HAL can be dropped in
 * without touching application code.
 */
#ifndef THUNDERS_STM32F4XX_HAL_CONF_H
#define THUNDERS_STM32F4XX_HAL_CONF_H

/* Oscillator values (must match system_stm32f4xx.h) */
#define HSE_VALUE    8000000U
#define HSI_VALUE    16000000U
#define LSE_VALUE    32768U
#define LSI_VALUE    32000U
#define EXTERNAL_SAI_CLOCK_VALUE 48000U
#define EXTERNAL_SAI2_CLOCK_VALUE 48000U

/* Tick priority: TIM6 update (lowest, below every application IRQ) */
#define TICK_INT_PRIORITY            15U

/* Stubs for features not used by the compact HAL */
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U
#define INSTRUCTION_CACHE_ENABLE     1U
#define DATA_CACHE_ENABLE            1U
#define USE_SPI_CRC                  0U

/* Compile-time asserts: none */
#define assert_param(expr)           ((void)0U)

#endif /* THUNDERS_STM32F4XX_HAL_CONF_H */
