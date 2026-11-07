/**
 * @file stm32f4xx_hal_spi.c
 * @brief Stub file for the upstream STM32F4xx HAL SPI implementation.
 *
 * The actual `HAL_SPI_Init`, `HAL_SPI_Transmit`, `HAL_SPI_Receive`,
 * `HAL_SPI_TransmitReceive`, and the rest of the STM32CubeF4 SPI HAL live in
 * the upstream tree at
 * `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_spi.c`.  That tree
 * is pulled in via a git submodule and added to the CMake build via the
 * `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_SPI_Transmit` so the project links
 * even without the submodule checked out.  When the upstream tree is present
 * the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_spi.h"

#include <stdio.h>

__attribute__((weak))
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
                                   uint8_t *pData,
                                   uint16_t Size,
                                   uint32_t Timeout)
{
    (void)pData; (void)Size; (void)Timeout;
    if (hspi == NULL) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi,
                                  uint8_t *pData,
                                  uint16_t Size,
                                  uint32_t Timeout)
{
    (void)Timeout;
    if (hspi == NULL || pData == NULL || Size == 0U) {
        return HAL_ERROR;
    }
    for (uint16_t i = 0; i < Size; i++) {
        pData[i] = 0xFFU;  /* typical SPI "no slave" pattern */
    }
    return HAL_OK;
}
