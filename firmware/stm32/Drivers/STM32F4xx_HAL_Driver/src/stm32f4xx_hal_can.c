/**
 * @file stm32f4xx_hal_can.c
 * @brief Stub file for the upstream STM32F4xx HAL CAN implementation.
 *
 * The actual `HAL_CAN_Init`, `HAL_CAN_Start`, `HAL_CAN_AddTxMessage`,
 * `HAL_CAN_GetRxMessage`, and the rest of the STM32CubeF4 CAN HAL live in the
 * upstream tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_can.c`.
 * That tree is pulled in via a git submodule and added to the CMake build via
 * the `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_CAN_GetError` so the project links
 * even without the submodule checked out.  When the upstream tree is present
 * the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_can.h"

#include <stdio.h>

__attribute__((weak))
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan->Instance == NULL) {
        return 0U;
    }
    /* Direct read of the CAN error status register. */
    return hcan->Instance->ESR;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
