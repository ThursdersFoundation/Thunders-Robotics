/**
 * @file stm32f4xx_hal_i2c.h
 * @brief Project integration re-export of the upstream STM32F4xx I2C HAL API.
 *
 * The actual `HAL_I2C_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_i2c.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This wrapper exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_i2c.h"` without
 * depending on a particular relative path.
 *
 * Project-specific I2C bus configuration (BNO055 IMU on I2C1 @ 100 kHz) lives
 * in `Core/Inc/i2c.h` and `Core/Src/i2c.c`.
 */
#ifndef STM32F4XX_HAL_I2C_H
#define STM32F4XX_HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream I2C HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_i2c.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_I2C_H */
