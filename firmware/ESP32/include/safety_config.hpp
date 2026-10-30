/**
 * @file safety_config.hpp
 * @brief Safety supervision thresholds and emergency-stop behavior.
 *
 * The safety layer is intentionally the simplest, most defensive part of the
 * firmware. It runs at the highest priority and may force the robot into
 * `RobotMode::kEmergency` whenever any threshold is exceeded. Every threshold
 * here should be set **conservatively** — false positives are annoying, but
 * false negatives can damage hardware or hurt people.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_SAFETY_CONFIG_HPP
#define THUNDERS_SAFETY_CONFIG_HPP

#include <cstdint>

namespace thunders::config::safety {

// ─── Watchdog ─────────────────────────────────────────────────────────────
inline constexpr std::uint32_t kWatchdogTimeoutMs   = 2'000;
inline constexpr std::uint32_t kWatchdogPanicGraceMs = 500;
inline constexpr std::uint16_t kWatchdogMaxResets    = 5;

// ─── Emergency-stop behavior ──────────────────────────────────────────────
inline constexpr std::uint32_t kEstopDebounceMs      = 10;     // ignore glitches
inline constexpr std::uint32_t kMotorsHaltPwmDurationMs = 50;   // coast ramp
inline constexpr bool         kRequireManualResetOnEstop = true; // never auto-recover
inline constexpr std::uint32_t kEstopSignalHoldMs    = 250;    // notify window

// ─── Velocity & acceleration limits (hard caps, override app config) ─────
inline constexpr float kSafeMaxLinearVelocityMs  = 0.45f;
inline constexpr float kSafeMaxAngularVelocityRs = 4.5f;
inline constexpr float kSafeMaxLinearAccelMs2    = 1.0f;
inline constexpr float kSafeMaxAngularAccelRadS2 = 5.0f;

// ─── Thermal / electrical protection ──────────────────────────────────────
inline constexpr float kMotorDriverOvertempC        = 75.0f;
inline constexpr float kCriticalBatteryVoltageV      = 6.6f;
inline constexpr float kCriticalBatteryVoltageHystV  = 0.2f;  // hysteresis
inline constexpr std::uint32_t kLowBatteryAutoHaltMs  = 5'000; // 5 s of low → halt

// ─── Collision avoidance (uses ultrasonic distance) ───────────────────────
inline constexpr float kCautionDistanceM     = 0.30f;  // slow down
inline constexpr float kDangerDistanceM       = 0.15f;  // must stop
inline constexpr float kStopDistanceM         = 0.08f;  // hard stop
inline constexpr std::uint32_t kStallDetectionWindowMs = 500;
inline constexpr float kStallVelocityThresholdMs = 0.02f;

// ─── IMU-based fall / tip-over detection ──────────────────────────────────
inline constexpr float kTipOverPitchRad   = 0.78f;  // ~45°
inline constexpr float kTipOverRollRad    = 0.78f;
inline constexpr std::uint32_t kTipOverDebounceMs = 100;

// ─── Heartbeat (remote link alive?) ──────────────────────────────────────
inline constexpr std::uint32_t kHeartbeatExpectedMs  = 1'000;
inline constexpr std::uint32_t kHeartbeatGraceMs    = 1'500;
inline constexpr std::uint8_t  kHeartbeatMissedBeforeHalt = 3;

}  // namespace thunders::config::safety

#endif  // THUNDERS_SAFETY_CONFIG_HPP
