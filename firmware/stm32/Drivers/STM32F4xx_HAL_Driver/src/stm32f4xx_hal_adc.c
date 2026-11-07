/**
 * @file stm32f4xx_hal_adc.c
 * @brief Stub file for the upstream STM32F4xx HAL ADC implementation.
 *
 * The actual `HAL_ADC_Init`, `HAL_ADC_Start`, `HAL_ADC_PollForConversion`,
 * `HAL_ADC_GetValue`, and the rest of the STM32CubeF4 ADC HAL live in the
 * upstream tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_adc.c`.
 * That tree is pulled in via a git submodule and added to the CMake build via
 * the `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_ADC_GetValue` so the project links
 * even without the submodule checked out.  When the upstream tree is present
 * the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_adc.h"

#include <stdio.h>

__attribute__((weak))
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance == NULL) {
        return 0U;
    }
    /* Direct read of the regular data register. */
    return hadc->Instance->DR;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
    (void)Timeout;
    if (hadc == NULL) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
