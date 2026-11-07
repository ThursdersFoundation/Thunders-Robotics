/**
 * @file stm32f4xx_hal.c
 * @brief Stub file for the upstream STM32F4xx HAL core implementation.
 *
 * The actual `HAL_Init`, `HAL_GetTick`, `HAL_Delay`, `HAL_GetTickPrio`, and
 * the rest of the STM32CubeF4 HAL core live in the upstream tree at
 * `Drivers/STM32F4xx_HAL_Driver/upstream/Src/stm32f4xx_hal.c`.  That tree is
 * pulled in via a git submodule and added to the CMake build via the
 * `STM32_HAL_DRIVER_DIR` variable defined in the top-level `CMakeLists.txt`.
 *
 * This stub file exists so that a build that has NOT yet checked out the
 * submodule still links, via the WEAK `HAL_GetTick` alias below.  When the
 * upstream tree is present the strong upstream definition overrides this.
 *
 * TODO: add `Drivers/STM32F4xx_HAL_Driver/upstream/` as a git submodule so
 *       this stub is replaced by the real implementation.
 */
#include "stm32f4xx_hal.h"

#include <stdio.h>

/* ITM stimulus port 0 - prints via SWO if the debugger is attached. */
#define ITM_SendChar8(c) \
    do { while (ITM->PORT[0].u32 == 0U) { } \
         ITM->PORT[0].u8 = (uint8_t)(c); } while (0)

__attribute__((weak))
uint32_t HAL_GetTick(void)
{
    /* Fallback when the upstream HAL is not linked in.  Uses the DWT cycle
     * counter for a rough monotonic tick at the assumed 168 MHz sysclk. */
    static uint32_t s_last_cycles = 0U;
    uint32_t now = DWT->CYCCNT;
    uint32_t delta = now - s_last_cycles;
    s_last_cycles = now;
    /* Convert cycles to ms at 168 MHz. */
    return delta / 168000U;
}

__attribute__((weak))
void HAL_Delay(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        __NOP();
    }
}
