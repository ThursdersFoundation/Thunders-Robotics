/**
 * @file watchdog.cpp
 * @brief Implementation of the watchdog supervisor.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "safety/watchdog.hpp"

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "diagnostics/logger.hpp"
#include "safety_config.hpp"

namespace thunders::safety {

namespace {
constexpr const char* kTag = "[wdog]";
}

Watchdog::~Watchdog() {
    if (initialised_) {
        esp_task_wdt_delete(nullptr);
    }
}

void Watchdog::begin() {
    using namespace thunders::config::safety;
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms        = kWatchdogTimeoutMs;
    cfg.idle_core_mask    = 0x03;  // subscribe both idle tasks
    cfg.trigger_panic     = true;
    esp_task_wdt_init(&cfg);
    esp_task_wdt_add(nullptr);

    lastFeedMs_ = millis();
    initialised_ = true;
    thunders::diagnostics::Logger::instance().info(
        "%s TWDT configured (timeout=%lu ms, panic on expiry)",
        kTag, static_cast<unsigned long>(kWatchdogTimeoutMs));
}

void Watchdog::feed() {
    if (!initialised_) return;
    esp_task_wdt_reset();
    lastFeedMs_ = millis();
}

}  // namespace thunders::safety
