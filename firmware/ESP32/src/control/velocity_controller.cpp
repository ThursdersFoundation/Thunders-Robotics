/**
 * @file velocity_controller.cpp
 * @brief Differential-drive inverse kinematics + ramped velocity control.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "control/velocity_controller.hpp"

#include <Arduino.h>
#include <algorithm>
#include <cmath>

#include "diagnostics/logger.hpp"
#include "motor_config.hpp"
#include "safety_config.hpp"

namespace thunders::control {

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr const char* kTag = "[vel_ctrl]";

/// Ramp `current` toward `target` by at most `step` per call.
inline float rampToward(float current, float target, float step) {
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}
}  // namespace

VelocityController::VelocityController(MotorController& m) : motor_(m) {}

void VelocityController::begin() {
    lastUpdateMs_ = millis();
    thunders::diagnostics::Logger::instance().info(
        "%s ready (safe lin=%.2f m/s ang=%.2f rad/s)",
        kTag, thunders::config::safety::kSafeMaxLinearVelocityMs,
        thunders::config::safety::kSafeMaxAngularVelocityRs);
}

void VelocityController::setTarget(float linear_mps, float angular_rps) {
    using namespace thunders::config::safety;
    targetLinear_  = std::clamp(linear_mps,
                                -kSafeMaxLinearVelocityMs,
                                +kSafeMaxLinearVelocityMs);
    targetAngular_ = std::clamp(angular_rps,
                                -kSafeMaxAngularVelocityRs,
                                +kSafeMaxAngularVelocityRs);
}

void VelocityController::update() {
    using namespace thunders::config::motor;

    const std::uint32_t now_ms = millis();
    const std::uint32_t dt_ms  = now_ms - lastUpdateMs_;
    lastUpdateMs_ = now_ms;
    if (dt_ms == 0) return;
    const float dt = static_cast<float>(dt_ms) * 0.001f;

    // Apply acceleration limits.
    const float linStep = kRampLinearMs2    * dt;
    const float angStep = kRampAngularRadS2 * dt;
    rampedLinear_  = rampToward(rampedLinear_,  targetLinear_,  linStep);
    rampedAngular_ = rampToward(rampedAngular_, targetAngular_, angStep);

    // Differential-drive inverse kinematics: per-wheel linear velocity.
    const float vL = (rampedLinear_ - rampedAngular_ * (kWheelbaseMeters * 0.5f))
                     / (kWheelDiameterMeters * 0.5f);
    const float vR = (rampedLinear_ + rampedAngular_ * (kWheelbaseMeters * 0.5f))
                     / (kWheelDiameterMeters * 0.5f);

    // Convert wheel angular velocity (rad/s) to RPM.
    const float rpmL = vL * 60.0f / (2.0f * kPi);
    const float rpmR = vR * 60.0f / (2.0f * kPi);

    motor_.setTargets(rpmL, rpmR);
}

void VelocityController::halt() {
    targetLinear_  = 0.0f;
    targetAngular_ = 0.0f;
    // Ramped ramp-down will happen over a few ticks; e-stop paths short-circuit
    // this by calling MotorController::halt() directly.
}

}  // namespace thunders::control
