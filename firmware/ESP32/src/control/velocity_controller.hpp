/**
 * @file velocity_controller.hpp
 * @brief Body-frame velocity controller (linear + angular → wheel RPM).
 *
 * Translates Twist-style commands (linear m/s, angular rad/s) into per-wheel
 * RPM targets for the underlying `MotorController`. Applies:
 *   - Velocity clamping to safe limits (safety_config.hpp).
 *   - Linear acceleration ramping (motor_config.hpp) for smooth teleop.
 *   - Differential-drive inverse kinematics:
 *       v_left  = (v - ω·L/2) / r_wheel
 *       v_right = (v + ω·L/2) / r_wheel
 *       rpm = v_wheel * 60 / (π · d)
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_VELOCITY_CONTROLLER_HPP
#define THUNDERS_VELOCITY_CONTROLLER_HPP

#include <cstdint>

#include "control/motor_controller.hpp"

namespace thunders::control {

class VelocityController {
public:
    explicit VelocityController(MotorController& motor);
    ~VelocityController() = default;

    /// Configure clamps & ramp rates from config headers.
    void begin();

    /// Set the body-frame target. Clamped to safe limits immediately.
    void setTarget(float linear_mps, float angular_rps);

    /// Read the (already-clamped) target back.
    struct Target { float linear; float angular; };
    Target target() const { return {targetLinear_, targetAngular_}; }

    /// Compute wheel RPM targets from the ramped body-frame target and push
    /// them into the underlying MotorController. Call from the control task
    /// at the configured period.
    void update();

    /// Soft stop: ramp linear and angular to zero.
    void halt();

private:
    MotorController& motor_;

    float targetLinear_  = 0.0f;  // m/s, clamped
    float targetAngular_ = 0.0f;  // rad/s, clamped
    float rampedLinear_  = 0.0f;
    float rampedAngular_ = 0.0f;
    std::uint32_t lastUpdateMs_ = 0;
};

}  // namespace thunders::control

#endif  // THUNDERS_VELOCITY_CONTROLLER_HPP
