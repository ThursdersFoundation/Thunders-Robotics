/**
 * @file health_monitor.cpp
 * @brief Health-monitor implementation: battery ADC, heap, uptime, safety
 *        thresholds (low battery, tip-over).
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "diagnostics/health_monitor.hpp"

#include <Arduino.h>

#include "app/robot_state.hpp"
#include "diagnostics/logger.hpp"
#include "pin_config.hpp"
#include "safety_config.hpp"
#include "sensor_config.hpp"

namespace thunders::diagnostics {

namespace {
constexpr const char* kTag = "[health]";
}

void HealthMonitor::begin() {
    using namespace thunders::pins;
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0..3.3 V full range
    thunders::diagnostics::Logger::instance().info(
        "%s ready (battery sense on GPIO %u)", kTag, kBatterySense);
}

void HealthMonitor::sampleBattery() {
    using namespace thunders::config::sensor::battery;
    using namespace thunders::pins;

    std::uint32_t acc = 0;
    for (std::uint8_t i = 0; i < kAdcSamplesAveraged; ++i) {
        acc += static_cast<std::uint32_t>(analogRead(kBatterySense));
    }
    const float avg = static_cast<float>(acc) /
                     static_cast<float>(kAdcSamplesAveraged);
    // 12-bit ADC, 3.3 V ref, voltage divider ratio.
    constexpr float kAdcRefV = 3.3f;
    constexpr float kAdcMax  = 4095.0f;
    const float v_adc = avg * (kAdcRefV / kAdcMax);
    batteryV_ = v_adc * kDividerRatio;

    app_.state().setBattery(batteryV_);

    // Latched low-battery detection: must persist for kLowBatteryAutoHaltMs.
    using namespace thunders::config::safety;
    if (batteryV_ < kCriticalBatteryVoltageV) {
        if (lowBattSinceMs_ == 0) lowBattSinceMs_ = millis();
        if ((millis() - lowBattSinceMs_) >= kLowBatteryAutoHaltMs && !lowBattLatched_) {
            lowBattLatched_ = true;
            thunders::diagnostics::Logger::instance().error(
                "%s battery %.2f V < %.2f V — triggering e-stop",
                kTag, batteryV_, kCriticalBatteryVoltageV);
            app_.estop().trigger(safety::EstopReason::kLowBattery);
        }
    } else if (batteryV_ > kCriticalBatteryVoltageV + kCriticalBatteryVoltageHystV) {
        // Recovered — clear the latch so we can detect a new dip later.
        lowBattSinceMs_ = 0;
        lowBattLatched_ = false;
    }
}

void HealthMonitor::update() {
    auto& state = app_.state();
    auto snap = state.snapshot();

    // Update diagnostic counters from the logger.
    const auto& log = thunders::diagnostics::Logger::instance();
    snap.errors   = log.errorCount();
    snap.warnings = log.warnCount();

    // Free heap.
    snap.freeHeapBytes = static_cast<float>(freeHeapBytes());
    state.setSnapshot(snap);

    // Tip-over detection from IMU (only if calibrated).
    if (snap.imuCalibrated) {
        using namespace thunders::config::safety;
        const bool tippedOver =
            (std::fabs(snap.pitchRad) > kTipOverPitchRad) ||
            (std::fabs(snap.rollRad)  > kTipOverRollRad);
        if (tippedOver && !snap.estopTriggered) {
            thunders::diagnostics::Logger::instance().error(
                "%s tip-over detected (pitch=%.2f roll=%.2f)",
                kTag, snap.pitchRad, snap.rollRad);
            app_.estop().trigger(safety::EstopReason::kTipOver);
        }
    }

    // Periodic health snapshot to the log (every ~10 s).
    static std::uint32_t lastLog = 0;
    if (millis() - lastLog > 10'000) {
        lastLog = millis();
        thunders::diagnostics::Logger::instance().info(
            "%s uptime=%lus heap=%u B batt=%.2f V mode=%s errors=%u warns=%u",
            kTag,
            static_cast<unsigned long>(uptimeSeconds()),
            static_cast<unsigned>(freeHeapBytes()),
            batteryV_,
            thunders::config::robotModeToString(snap.mode),
            static_cast<unsigned>(snap.errors),
            static_cast<unsigned>(snap.warnings));
    }
}

std::uint32_t HealthMonitor::freeHeapBytes() const {
    return ESP.getFreeHeap();
}

std::uint32_t HealthMonitor::uptimeSeconds() const {
    return static_cast<std::uint32_t>(millis() / 1000);
}

}  // namespace thunders::diagnostics
