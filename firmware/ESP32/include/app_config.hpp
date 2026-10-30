/**
 * @file app_config.hpp
 * @brief Top-level application configuration for Thunders Robotic firmware.
 *
 * Central place for high-level behavioural parameters: tick rates, mode
 * definitions, version strings, and feature toggles. Everything here is
 * `constexpr` so the compiler can fold it at compile-time and emit warnings
 * for unused or unreachable branches.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_APP_CONFIG_HPP
#define THUNDERS_APP_CONFIG_HPP

#include <cstdint>

namespace thunders::config {

// ─── Firmware metadata ─────────────────────────────────────────────────────
inline constexpr const char* kFirmwareVersion   = "0.1.0";
inline constexpr const char* kFirmwareBuildDate  = __DATE__;
inline constexpr const char* kFirmwareBuildTime = __TIME__;
inline constexpr const char* kRobotName         = "thunders-001";

// ─── Task scheduling (FreeRTOS tick period in ms) ──────────────────────────
inline constexpr std::uint32_t kTickPeriodMs        = 10;     // 100 Hz
inline constexpr std::uint32_t kControlLoopPeriodMs  = 10;     // 100 Hz
inline constexpr std::uint32_t kTelemetryPeriodMs     = 100;    // 10 Hz
inline constexpr std::uint32_t kHealthMonitorPeriodMs = 1000;  // 1 Hz
inline constexpr std::uint32_t kWatchdogTimeoutMs    = 2000;   // 2 s

// ─── FreeRTOS task priorities (higher = more important) ────────────────────
inline constexpr std::uint32_t kPrioSafety      = 24;  // e-stop must preempt
inline constexpr std::uint32_t kPrioControl     = 20;  // closed-loop control
inline constexpr std::uint32_t kPrioDrivers     = 16;  // sensor sampling
inline constexpr std::uint32_t kPrioComms       = 12;  // MQTT / Wi-Fi
inline constexpr std::uint32_t kPrioTelemetry   = 8;   // periodic logging
inline constexpr std::uint32_t kPrioDiagnostics  = 4;   // health monitor
inline constexpr std::uint32_t kPrioIdle        = 0;   // IDLE task

// ─── FreeRTOS task stack sizes (in 32-bit words, not bytes) ────────────────
inline constexpr std::uint32_t kStackApp        = 4096;
inline constexpr std::uint32_t kStackControl    = 2048;
inline constexpr std::uint32_t kStackComms      = 6144;  // Wi-Fi + TLS is heavy
inline constexpr std::uint32_t kStackDiagnostics = 2048;
inline constexpr std::uint32_t kStackSafety     = 1024;

// ─── Feature toggles ──────────────────────────────────────────────────────
inline constexpr bool kEnableWifi      = true;
inline constexpr bool kEnableMqtt      = true;
inline constexpr bool kEnableRosBridge = false;  // experimental
inline constexpr bool kEnableOta       = true;
inline constexpr bool kEnableImu       = true;
inline constexpr bool kEnableUltrasonic = true;
inline constexpr bool kEnableWatchdog  = true;

// ─── Robot operating modes ────────────────────────────────────────────────
enum class RobotMode : std::uint8_t {
    kBoot        = 0,  // initialising, actuators disabled
    kIdle        = 1,  // ready, actuators disabled
    kTeleop      = 2,  // commanded remotely, motors enabled
    kAutonomous  = 3,  // running onboard behaviour
    kHalt        = 4,  // soft stop: zero velocity, motors coast
    kEmergency   = 5,  // hard stop: motors disabled, await reset
};

inline constexpr const char* robotModeToString(RobotMode m) {
    switch (m) {
        case RobotMode::kBoot:       return "BOOT";
        case RobotMode::kIdle:       return "IDLE";
        case RobotMode::kTeleop:     return "TELEOP";
        case RobotMode::kAutonomous: return "AUTONOMOUS";
        case RobotMode::kHalt:       return "HALT";
        case RobotMode::kEmergency:  return "EMERGENCY";
    }
    return "UNKNOWN";
}

}  // namespace thunders::config

#endif  // THUNDERS_APP_CONFIG_HPP
