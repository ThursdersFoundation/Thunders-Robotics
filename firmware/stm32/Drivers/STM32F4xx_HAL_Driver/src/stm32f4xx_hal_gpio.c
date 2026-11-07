/**
 * @file stm32f4xx_hal_gpio.c
 * @brief Stub file for the upstream STM32F4xx HAL GPIO implementation.
 *
 * The actual `HAL_GPIO_Init`, `HAL_GPIO_WritePin`, `HAL_GPIO_ReadPin`,
 * `HAL_GPIO_DeInit`, and `HAL_GPIO_EXTI_IRQHandler` live in the upstream tree
 * at `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_gpio.c`.  That
 * tree is pulled in via a git submodule and added to the CMake build via the
 * `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_GPIO_WritePin` so the project links
 * even without the submodule checked out.  When the upstream tree is present
 * the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_gpio.h"

#include <stdio.h>

__attribute__((weak))
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if (GPIOx == NULL) {
        return;
    }
    /* Direct BSRR write so the stub is still useful for debugging without
     * the upstream HAL (BSRR is a constant offset on all STM32 families). */
    if (PinState == GPIO_PIN_SET) {
        GPIOx->BSRR = (uint32_t)GPIO_Pin;
    } else {
        GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
    }
}

__attribute__((weak))
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    if (GPIOx == NULL) {
        return GPIO_PIN_RESET;
    }
    return ((GPIOx->IDR & GPIO_Pin) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
