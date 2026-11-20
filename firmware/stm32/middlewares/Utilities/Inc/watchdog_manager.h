/**
 * @file    watchdog_manager.h
 * @brief   Independent watchdog (IWDG) management + task supervision hooks.
 *
 * IWDG runs from the LSI (~32 kHz) and cannot be stopped once started, so
 * the manager only starts it after all subsystem inits succeeded. The
 * safety task refreshes the counter every cycle; any hang of the scheduler
 * or of the safety task itself resets the MCU into the SAFE_STOP boot state
 * (motors are PWM-gated off by hardware until re-enabled).
 */
#ifndef THUNDERS_WATCHDOG_MANAGER_H
#define THUNDERS_WATCHDOG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool     started;
    uint32_t timeout_ms;
    uint32_t refresh_count;
} WatchdogManager;

/** Start the IWDG with the given timeout (32..32000 ms, LSI-based). */
bool Watchdog_Start(uint32_t timeout_ms);
/** Refresh (kick) the watchdog. Call only from the safety task. */
void Watchdog_Refresh(void);
/** True once the IWDG has been started. */
bool Watchdog_IsStarted(void);
/**
 * Configure RCC/option bytes so that after an IWDG reset the system boots
 * with motors disarmed (reads the reset-cause register).
 */
bool Watchdog_WasResetByWatchdog(void);

#ifdef __cplusplus
}
#endif

#endif /* THUNDERS_WATCHDOG_MANAGER_H */
