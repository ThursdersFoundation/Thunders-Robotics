/**
 * @file stm32f4xx_hal_uart.c
 * @brief Stub file for the upstream STM32F4xx HAL UART implementation.
 *
 * The actual `HAL_UART_Init`, `HAL_UART_Transmit`, `HAL_UART_Receive`,
 * `HAL_UARTEx_ReceiveToIdle_DMA`, and the rest of the STM32CubeF4 UART HAL
 * live in the upstream tree at
 * `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal_uart.c`.  That tree
 * is pulled in via a git submodule and added to the CMake build via the
 * `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub provides a WEAK alias of `HAL_UART_Transmit` so the project
 * links even without the submodule checked out.  When the upstream tree is
 * present the strong upstream definition overrides this weak alias.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal_uart.h"

#include <stdio.h>

/* ITM stimulus port 0 - prints the UART2 debug log via SWO if attached. */
static void itm_send(const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        while (ITM->PORT[0].u32 == 0U) { }
        ITM->PORT[0].u8 = p[i];
    }
}

__attribute__((weak))
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout)
{
    (void)Timeout;
    if (huart == NULL || pData == NULL) {
        return HAL_ERROR;
    }
    /* Mirror debug output to ITM/SW0 so a no-submodule build still gives
     * the developer a log of writes. */
    itm_send(pData, Size);
    return HAL_OK;
}

__attribute__((weak))
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout)
{
    (void)Timeout;
    if (huart == NULL || pData == NULL || Size == 0U) {
        return HAL_ERROR;
    }
    for (uint16_t i = 0; i < Size; i++) {
        pData[i] = 0U;
    }
    return HAL_OK;
}
