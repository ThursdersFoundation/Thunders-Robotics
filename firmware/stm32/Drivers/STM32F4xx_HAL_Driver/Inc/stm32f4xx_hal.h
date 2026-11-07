/**
 * @file stm32f4xx_hal.h
 * @brief Project integration re-export of the upstream STM32F4xx HAL master
 *        header.
 *
 * The Thunders Robotic firmware does NOT vendor the upstream STM32F4xx HAL
 * driver tree.  Instead, the STM32CubeF4 HAL is pulled in via a git submodule
 * rooted at `Drivers/STM32F4xx_HAL_Driver/upstream/` and added to the CMake
 * include path via the `STM32_HAL_DRIVER_DIR` variable (see the top-level
 * `CMakeLists.txt`).  This thin wrapper file lets the rest of the codebase
 * write `#include "stm32f4xx_hal.h"` without caring about the upstream
 * checkout location, and provides a single anchor point if the firmware ever
 * needs to add a project-wide shim macro on top of the HAL.
 *
 * To initialise the submodule:
 *
 *   git submodule add \
 *       https://github.com/STMicroelectronics/STM32CubeF4 \
 *       Drivers/STM32F4xx_HAL_Driver/upstream
 *   git -C Drivers/STM32F4xx_HAL_Driver/upstream \
 *       checkout v1.27.1
 *
 * The per-peripheral headers (stm32f4xx_hal_gpio.h, _adc.h, _can.h, etc.)
 * follow the same pattern; each forwards the corresponding upstream header.
 */
#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Pull in the real upstream HAL master header from STM32_HAL_DRIVER_DIR/Inc. */
#include "../../upstream/Inc/stm32f4xx_hal.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_H */
