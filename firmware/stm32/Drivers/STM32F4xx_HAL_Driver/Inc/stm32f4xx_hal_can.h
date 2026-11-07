/**
 * @file stm32f4xx_hal_can.h
 * @brief Project integration re-export of the upstream STM32F4xx CAN HAL API.
 *
 * The actual `HAL_CAN_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_can.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This wrapper exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_can.h"` without
 * depending on a particular relative path.
 *
 * Project-specific CAN bit timing and the standard-frame message ID table
 * live in `Core/Inc/can.h` and `Middlewares/Communication/can_protocol.h`.
 */
#ifndef STM32F4XX_HAL_CAN_H
#define STM32F4XX_HAL_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream CAN HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_can.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_CAN_H */
