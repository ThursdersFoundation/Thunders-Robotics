// =============================================================================
// thunders/control/velocity_controller.cpp
// -----------------------------------------------------------------------------
// Closed-loop velocity controller wrapping a MotorController.
//
// Uses a PID controller on velocity error plus a simple feedforward term
// (kV * target + kS) to provide the bulk of the motor output. The PID only
// handles transient error, which dramatically reduces integral wind-up.
//
// Units: rad/s for both target and measurement. The output is a voltage sent
// to the wrapped MotorController in VOLTAGE mode.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include "thunders/control/pid_controller.cpp"

namespace thunders::control {

class MotorController;  // forward-declared in motor_controller.cpp

class VelocityController {
public:
    VelocityController(double kp, double ki, double kd,
                       double ks, double kv)
        : pid_(kp, ki, kd), ks_(ks), kv_(kv) {
        pid_.set_izone(2.0);
        pid_.set_max_output(12.0);
    }

    /// Sets the target velocity in rad/s.
    void set_target_velocity(double v) {
        target_ = v;
        pid_.set_setpoint(v);
    }

    /// Computes the motor output given the current measured velocity.
    /// The caller is responsible for forwarding the output to the
    /// MotorController (typically via set_voltage).
    double update(double measured_velocity_rad_per_s) {
        const double ff = ks_ * std::copysign(1.0, target_) + kv_ * target_;
        const double pid_out = pid_.calculate(measured_velocity_rad_per_s);
        return ff + pid_out;
    }

    /// Resets the integral and derivative state. Call when the controller
    /// is re-enabled after being disabled.
    void reset() { pid_.reset(); }

    bool at_target(double measured, double tol) const {
        return pid_.at_setpoint(measured, tol);
    }

    // --- tunables --------------------------------------------------------
    void set_gains(double kp, double ki, double kd) { pid_.set_gains(kp, ki, kd); }
    void set_feedforward(double ks, double kv) { ks_ = ks; kv_ = kv; }
    void set_max_voltage(double v) { pid_.set_max_output(v); }

    // --- diagnostics -----------------------------------------------------
    double target() const noexcept { return target_; }
    double integral() const noexcept { return pid_.integral(); }

private:
    PIDController pid_;
    double        ks_      {0.0};   ///< Static friction feedforward (volts).
    double        kv_      {0.0};   ///< Velocity feedforward (volts per rad/s).
    double        target_  {0.0};
};

} // namespace thunders::control
