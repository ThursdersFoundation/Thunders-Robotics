/**
 * @file stm32f4xx_hal_spi.h
 * @brief Project integration re-export of the upstream STM32F4xx SPI HAL API.
 *
 * The actual `HAL_SPI_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_spi.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This wrapper exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_spi.h"` without
 * depending on a particular relative path.
 *
 * Project-specific SPI bus configuration (master, mode 0, 8-bit, 10.5 MHz
 * on PA5/PA6/PA7 with software CS on PB2) lives in `Core/Inc/spi.h` and
 * `Core/Src/spi.c`.
 */
#ifndef STM32F4XX_HAL_SPI_H
#define STM32F4XX_HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream SPI HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_spi.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_SPI_H */
