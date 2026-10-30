/**
 * @file motor_config.hpp
 * @brief Mechanical and electrical parameters of the drive motors.
 *
 * These constants describe the physical motor/gearbox/encoder system and
 * the closed-loop gains used by `motor_controller.cpp`. Tuning is best done
 * empirically: start with conservative gains, double P until oscillation,
 * back off, then add I to remove steady-state error.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_MOTOR_CONFIG_HPP
#define THUNDERS_MOTOR_CONFIG_HPP

#include <cstdint>

namespace thunders::config::motor {

// ─── Geometry (differential-drive robot) ─────────────────────────────────
inline constexpr float kWheelDiameterMeters   = 0.065f;  // 65 mm
inline constexpr float kWheelbaseMeters       = 0.180f;  // 180 mm
inline constexpr float kWheelCircumferenceM   = 3.14159265358979f * kWheelDiameterMeters;
inline constexpr float kCountsPerRevolution   = 1440.0f; // 360 PPR × 4 (quadrature)
inline constexpr float kCountsPerRadian       = kCountsPerRevolution / (2.0f * 3.14159265358979f);
inline constexpr float kMaxLinearVelocityMs   = 0.500f;  // 0.5 m/s safe cap
inline constexpr float kMaxAngularVelocityRs = 5.0f;     // ~286 °/s safe cap

// ─── PWM limits (10-bit resolution, see board_config.hpp) ─────────────────
inline constexpr std::uint16_t kPwmDeadband   = 30;    // below this, motor stalls
inline constexpr std::uint16_t kPwmMax        = 1023;  // 100 % duty
inline constexpr std::uint16_t kPwmAccelerationLimit = 50;  // per control tick (10 ms)

// ─── PID gains for the closed-loop motor speed controller ────────────────
// Gains expressed in (pwm / (counts/s error)).
// Tuned for a 6 V 250 rpm geared motor with 1440 CPR quadrature encoder.
inline constexpr float kVelocityKp = 8.0f;
inline constexpr float kVelocityKi = 0.5f;
inline constexpr float kVelocityKd = 0.05f;
inline constexpr float kVelocityOutputLimit = static_cast<float>(kPwmMax);
inline constexpr float kVelocityIntegralLimit = kVelocityOutputLimit * 0.4f;

// ─── Drive-direction sign (set to -1 if a motor runs backwards) ───────────
inline constexpr std::int8_t kLeftDirSign  = +1;
inline constexpr std::int8_t kRightDirSign = -1;  // mirrored mounting

// ─── Open-loop ramp used during teleop (smoother joystick response) ────────
inline constexpr float kRampLinearMs2     = 1.5f;  // m/s²
inline constexpr float kRampAngularRadS2  = 8.0f;  // rad/s²

}  // namespace thunders::config::motor

#endif  // THUNDERS_MOTOR_CONFIG_HPP
