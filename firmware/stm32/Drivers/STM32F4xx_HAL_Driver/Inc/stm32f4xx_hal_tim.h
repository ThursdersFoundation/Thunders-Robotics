/**
 * @file stm32f4xx_hal_tim.h
 * @brief Project integration re-export of the upstream STM32F4xx TIM HAL API.
 *
 * The actual `HAL_TIM_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_tim.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This wrapper exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_tim.h"` without
 * depending on a particular relative path.
 *
 * Project-specific timer assignments (TIM1 motor PWM @ 20 kHz, TIM2/TIM5
 * encoders, TIM3 100 Hz control loop, TIM6 1 kHz FreeRTOS tick) live in
 * `Core/Inc/tim.h` and `Core/Src/tim.c`.
 */
#ifndef STM32F4XX_HAL_TIM_H
#define STM32F4XX_HAL_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream TIM HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_tim.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_TIM_H */
