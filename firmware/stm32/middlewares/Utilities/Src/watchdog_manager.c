/**
 * @file    watchdog_manager.c
 * @brief   IWDG configuration: LSI 32 kHz / 256 prescaler -> 125 Hz counter.
 *
 * Timeout range with the 12-bit reload register: 8 ms .. 32.7 s.
 *   timeout_ms ~= reload * 8 ms (prescaler 256, LSI 32 kHz)
 */
#include "watchdog_manager.h"
#include "stm32f4xx_hal.h"

static WatchdogManager s_wdg = { false, 0U, 0U };

bool Watchdog_Start(uint32_t timeout_ms)
{
    if (timeout_ms < 8U)
    {
        timeout_ms = 8U;
    }
    if (timeout_ms > 32760U)
    {
        timeout_ms = 32760U;
    }

    uint32_t reload = timeout_ms / 8U;   /* 125 Hz counter */

    /* Enable LSI and wait for it to stabilize */
    RCC->CSR |= (1UL << 0);              /* LSION */
    while (!(RCC->CSR & (1UL << 1)))     /* LSIRDY */
    {
    }

    IWDG->KR = IWDG_KEY_ENABLE;          /* start the watchdog */
    IWDG->KR = IWDG_KEY_WRITE_ACCESS;    /* unlock PR/RLR */
    IWDG->PR  = 0x07U;                   /* prescaler /256 */
    IWDG->RLR = reload;
    IWDG->KR = IWDG_KEY_REFRESH;         /* reload the counter */

    /* Wait for the new values to be taken into account */
    while (IWDG->SR != 0U)
    {
    }

    s_wdg.started = true;
    s_wdg.timeout_ms = timeout_ms;
    return true;
}

void Watchdog_Refresh(void)
{
    if (s_wdg.started)
    {
        IWDG->KR = IWDG_KEY_REFRESH;
        s_wdg.refresh_count++;
    }
}

bool Watchdog_IsStarted(void)
{
    return s_wdg.started;
}

bool Watchdog_WasResetByWatchdog(void)
{
    /* RCC_CSR bit 29 = IWDGRSTF (independent watchdog reset flag) */
    uint32_t csr = RCC->CSR;
    RCC->CSR |= (1UL << 24);   /* RMVF: clear reset flags */
    return (csr & (1UL << 29)) != 0U;
}
