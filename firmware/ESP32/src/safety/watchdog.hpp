/**
 * @file watchdog.hpp
 * @brief Hardware watchdog + cooperative software watchdog.
 *
 * Combines the ESP32 Task Watchdog Timer (TWDT) with a per-task software
 * watchdog. Each long-lived task calls `feed()` periodically; if a task
 * stops feeding for longer than `kWatchdogTimeoutMs` the firmware panics
 * with a stack trace and reboots.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_WATCHDOG_HPP
#define THUNDERS_WATCHDOG_HPP

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace thunders::safety {

class Watchdog {
public:
    Watchdog() = default;
    ~Watchdog();

    /// Configure the TWDT and reset the per-task counters.
    void begin();

    /// Pet the watchdog. Should be called by every long-lived task at its
    /// expected rate. The control task, telemetry task, etc. all call this.
    void feed();

    /// Total number of watchdog-triggered resets since boot.
    std::uint32_t resetCount() const { return resets_; }

private:
    bool          initialised_ = false;
    std::uint32_t resets_      = 0;
    std::uint32_t lastFeedMs_  = 0;
};

}  // namespace thunders::safety

#endif  // THUNDERS_WATCHDOG_HPP
