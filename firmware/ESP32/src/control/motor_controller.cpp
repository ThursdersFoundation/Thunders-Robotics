/**
 * @file motor_controller.cpp
 * @brief Per-wheel PID velocity controller implementation.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "control/motor_controller.hpp"

#include <Arduino.h>

#include "diagnostics/logger.hpp"
#include "motor_config.hpp"

namespace thunders::control {

namespace {
constexpr const char* kTag = "[motor_ctrl]";
}

MotorController::MotorController(drivers::MotorDriver& m, drivers::EncoderDriver& e)
    : motor_(m), enc_(e) {}

void MotorController::begin() {
    using namespace thunders::config::motor;

    PidController::Gains g{kVelocityKp, kVelocityKi, kVelocityKd};
    pidLeft_.configure (g, -kVelocityOutputLimit, +kVelocityOutputLimit,
                        kVelocityIntegralLimit);
    pidRight_.configure(g, -kVelocityOutputLimit, +kVelocityOutputLimit,
                        kVelocityIntegralLimit);
    lastUpdateMs_ = millis();
    thunders::diagnostics::Logger::instance().info(
        "%s PIDs configured Kp=%.3f Ki=%.3f Kd=%.3f",
        kTag, kVelocityKp, kVelocityKi, kVelocityKd);
}

void MotorController::setTargets(float left_rpm, float right_rpm) {
    targetLeft_  = left_rpm;
    targetRight_ = right_rpm;
}

void MotorController::update() {
    const std::uint32_t now_ms = millis();
    const std::uint32_t dt_ms  = now_ms - lastUpdateMs_;
    lastUpdateMs_ = now_ms;
    if (dt_ms == 0) return;
    const float dt_s = static_cast<float>(dt_ms) * 0.001f;

    // Read fresh RPM from the encoders.
    const auto rpm = enc_.readRpm(dt_ms);

    // Compute PWM outputs via the per-wheel PID controllers.
    const float u_left  = pidLeft_.update (targetLeft_,  rpm.left,  dt_s);
    const float u_right = pidRight_.update(targetRight_, rpm.right, dt_s);

    // Apply deadband so the motors don't whine at low PWM.
    using namespace thunders::config::motor;
    auto apply = [](float u) -> std::int16_t {
        const float a = std::fabs(u);
        if (a < kPwmDeadband) return 0;
        return static_cast<std::int16_t>(std::clamp(
            u, -static_cast<float>(kPwmMax), +static_cast<float>(kPwmMax)));
    };
    motor_.setLeft (apply(u_left));
    motor_.setRight(apply(u_right));
}

void MotorController::halt() {
    targetLeft_  = 0.0f;
    targetRight_ = 0.0f;
    pidLeft_.reset();
    pidRight_.reset();
    motor_.setLeft(0);
    motor_.setRight(0);
}

}  // namespace thunders::control
