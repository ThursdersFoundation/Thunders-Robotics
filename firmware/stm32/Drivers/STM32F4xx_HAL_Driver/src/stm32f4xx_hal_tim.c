/**
 * @file stm32f4xx_hal_tim.c
 * @brief Stub file for the upstream STM32F4xx HAL TIM implementation.
 *
 * The actual `HAL_TIM_Base_Init`, `HAL_TIM_PWM_Init`, `HAL_TIM_Encoder_Init`,
 * `HAL_TIM_Base_Start_IT`, `HAL_TIM_PWM_Start`, `HAL_TIM_Encoder_Start`, and
 * the rest of the STM32CubeF4 TIM HAL live in the upstream tree at
 * `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_tim.c`.  That tree
 * is pulled in via a git submodule and added to the CMake build via the
 * `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_TIM_Base_Start_IT` so the project
 * links even without the submodule checked out.  When the upstream tree is
 * present the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_tim.h"

#include <stdio.h>

__attribute__((weak))
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return HAL_ERROR;
    }
    /* No-op until the upstream TIM HAL is linked in. */
    return HAL_OK;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)Channel;
    if (htim == NULL) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)Channel;
    if (htim == NULL) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
