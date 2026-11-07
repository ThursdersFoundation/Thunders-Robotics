/**
 * @file stm32f4xx_hal_adc.h
 * @brief Project integration re-export of the upstream STM32F4xx ADC HAL API.
 *
 * The actual `HAL_ADC_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_adc.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This wrapper exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_adc.h"` without
 * depending on a particular relative path.
 *
 * Project-specific ADC channel assignments (battery monitor on ADC1_IN0)
 * live in `Core/Inc/adc.h` and `Core/Src/adc.c`.
 */
#ifndef STM32F4XX_HAL_ADC_H
#define STM32F4XX_HAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream ADC HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_adc.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_ADC_H */
