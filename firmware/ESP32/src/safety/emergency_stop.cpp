/**
 * @file emergency_stop.cpp
 * @brief E-stop supervision implementation.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "safety/emergency_stop.hpp"

#include <Arduino.h>

#include "app/robot_state.hpp"  // (forwarded by callers; full include in cpp)
#include "diagnostics/logger.hpp"
#include "pin_config.hpp"
#include "safety_config.hpp"

namespace thunders::safety {

namespace {
constexpr const char* kTag = "[estop]";

const char* reasonToString(EstopReason r) {
    switch (r) {
        case EstopReason::kNone:             return "NONE";
        case EstopReason::kUserButton:        return "USER_BUTTON";
        case EstopReason::kUserCommand:       return "USER_COMMAND";
        case EstopReason::kLostHeartbeat:     return "LOST_HEARTBEAT";
        case EstopReason::kLowBattery:        return "LOW_BATTERY";
        case EstopReason::kTipOver:           return "TIP_OVER";
        case EstopReason::kObstacleStop:       return "OBSTACLE_STOP";
        case EstopReason::kWatchdogTriggered: return "WATCHDOG";
        case EstopReason::kUserShutdown:      return "SHUTDOWN";
    }
    return "UNKNOWN";
}
}  // namespace

void EmergencyStop::begin() {
    using namespace thunders::pins;
    pinMode(kEstopInput, INPUT_PULLUP);
    lastButtonStateMs_ = millis();
    lastButtonRaw_     = (digitalRead(kEstopInput) == HIGH);
    armed_             = true;
    thunders::diagnostics::Logger::instance().info(
        "%s armed on GPIO %d (active-low, pull-up)",
        kTag, kEstopInput);
}

void EmergencyStop::poll() {
    if (!armed_) return;

    // Hardware e-stop: debounced active-low.
    using namespace thunders::pins;
    using namespace thunders::config::safety;

    const bool raw = (digitalRead(kEstopInput) == HIGH);
    if (raw != lastButtonRaw_) {
        lastButtonRaw_     = raw;
        lastButtonStateMs_ = millis();
    } else if (!raw && !triggered_ &&
               (millis() - lastButtonStateMs_) >= kEstopDebounceMs) {
        trigger(EstopReason::kUserButton);
    }

    // (Software triggers — low battery, tip-over, obstacle, heartbeat — are
    // evaluated by the HealthMonitor and forwarded to this->trigger(...) when
    // a threshold is breached. Keeping them out of the e-stop poll loop avoids
    // duplicate sensor reads.)
}

void EmergencyStop::trigger(EstopReason reason) {
    if (triggered_ && reason_ == reason) return;
    triggered_  = true;
    reason_    = reason;
    triggerMs_ = millis();
    armed_     = false;

    // Hard stop: zero targets and disable motor enable line.
    velCtrl_.halt();
    motor_.disableAll();

    thunders::diagnostics::Logger::instance().error(
        "%s ESTOP triggered: %s — motors disabled",
        kTag, reasonToString(reason));

    // Optionally pulse the status LED as a visual indicator.
    digitalWrite(thunders::pins::kStatusLed, LOW);
    delay(50);
    digitalWrite(thunders::pins::kStatusLed, HIGH);
}

void EmergencyStop::reset() {
    if (thunders::config::safety::kRequireManualResetOnEstop) {
        thunders::diagnostics::Logger::instance().warn(
            "%s reset blocked — manual reset required", kTag);
        return;
    }
    triggered_ = false;
    reason_    = EstopReason::kNone;
    armed_     = true;
    thunders::diagnostics::Logger::instance().info("%s re-armed", kTag);
}

}  // namespace thunders::safety
