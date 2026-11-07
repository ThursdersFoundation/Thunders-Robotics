/**
 * @file stm32f4xx_hal_gpio.h
 * @brief Project integration re-export of the upstream STM32F4xx GPIO HAL API.
 *
 * The actual `HAL_GPIO_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_gpio.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This header exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_gpio.h"` without
 * depending on a particular relative path.
 *
 * Project-specific GPIO pin assignments (LEDs, ESTOP, motor direction/enable,
 * motor fault input) live in `Core/Inc/gpio.h` and `Core/Src/gpio.c`.
 */
#ifndef STM32F4XX_HAL_GPIO_H
#define STM32F4XX_HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream GPIO HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_gpio.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_GPIO_H */
