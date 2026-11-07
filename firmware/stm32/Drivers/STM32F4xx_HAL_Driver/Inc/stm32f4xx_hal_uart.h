/**
 * @file stm32f4xx_hal_uart.h
 * @brief Project integration re-export of the upstream STM32F4xx UART/USART
 *        HAL API.
 *
 * The actual `HAL_UART_*` declarations live in the upstream STM32CubeF4 HAL
 * tree at `Drivers/STM32F4xx_HAL_Driver/upstream/Inc/stm32f4xx_hal_uart.h`.
 * That tree is pulled in via git submodule and exposed through the CMake
 * variable `STM32_HAL_DRIVER_DIR`.  This wrapper exists so the project's
 * source files can simply write `#include "stm32f4xx_hal_uart.h"` without
 * depending on a particular relative path.
 *
 * Project-specific UART assignments (USART2 debug @ 115200, USART3 host @
 * 115200 with DMA, UART4 LIDAR @ 460800) live in `Core/Inc/usart.h` and
 * `Core/Src/usart.c`.
 */
#ifndef STM32F4XX_HAL_UART_H
#define STM32F4XX_HAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_conf.h"

/* Re-export the upstream UART HAL declarations. */
#include "../../upstream/Inc/stm32f4xx_hal_uart.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_UART_H */
