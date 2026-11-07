/**
 * @file stm32f4xx_hal_i2c.c
 * @brief Stub file for the upstream STM32F4xx HAL I2C implementation.
 *
 * The actual `HAL_I2C_Init`, `HAL_I2C_Master_Transmit`,
 * `HAL_I2C_Master_Receive`, `HAL_I2C_Mem_Read`, and the rest of the
 * STM32CubeF4 I2C HAL live in the upstream tree at
 * `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_i2c.c`.  That tree
 * is pulled in via a git submodule and added to the CMake build via the
 * `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_I2C_Master_Transmit` so the
 * project links even without the submodule checked out.  When the upstream
 * tree is present the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_i2c.h"

#include <stdio.h>

__attribute__((weak))
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
                                          uint16_t DevAddress,
                                          uint8_t *pData,
                                          uint16_t Size,
                                          uint32_t Timeout)
{
    (void)DevAddress; (void)pData; (void)Size; (void)Timeout;
    if (hi2c == NULL) {
        return HAL_ERROR;
    }
    /* No-op until the upstream I2C HAL is linked in. */
    return HAL_OK;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
                                          uint16_t DevAddress,
                                          uint8_t *pData,
                                          uint16_t Size,
                                          uint32_t Timeout)
{
    (void)DevAddress; (void)Timeout;
    if (hi2c == NULL || pData == NULL || Size == 0U) {
        return HAL_ERROR;
    }
    /* Stub returns zeroed data so callers do not see garbage. */
    for (uint16_t i = 0; i < Size; i++) {
        pData[i] = 0U;
    }
    return HAL_OK;
}
