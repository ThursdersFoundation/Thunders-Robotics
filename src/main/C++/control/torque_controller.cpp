// =============================================================================
// thunders/control/torque_controller.cpp
// -----------------------------------------------------------------------------
// Closed-loop torque controller.
//
// Used for mechanisms that must apply a precise force (e.g., a gripper that
// must squeeze a game piece without crushing it). Converts the desired
// torque to a motor current using the motor's torque constant, then sends
// the current command to the wrapped MotorController.
//
// The torque estimate is derived from the motor's measured current and the
// motor's torque constant K_t (N*m per amp). On most FRC motors K_t is
// documented in the datasheet; for brushed motors K_t ≈ K_v / 1000.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>

#include "thunders/control/pid_controller.cpp"

namespace thunders::control {

class TorqueController {
public:
    TorqueController(double kp, double ki, double kd,
                     double torque_constant, double max_torque)
        : pid_(kp, ki, kd),
          kt_(torque_constant),
          max_torque_(max_torque) {
        pid_.set_izone(0.5);
        pid_.set_max_output(max_torque);
    }

    /// Sets the target torque in N*m.
    void set_target_torque(double t) {
        target_ = std::clamp(t, -max_torque_, max_torque_);
        pid_.set_setpoint(target_);
    }

    /// Computes the motor current (in amps) required to achieve the
    /// target torque given the current measured torque.
    double update(double measured_torque) {
        const double torque_cmd = pid_.calculate(measured_torque);
        return torque_cmd / std::max(kt_, 1e-6);
    }

    /// Convenience: sets the target as a fraction of max torque.
    void set_target_fraction(double f) {
        set_target_torque(f * max_torque_);
    }

    void reset() { pid_.reset(); }

    bool at_target(double measured, double tol) const {
        return pid_.at_setpoint(measured, tol);
    }

    // --- tunables --------------------------------------------------------
    void set_gains(double kp, double ki, double kd) { pid_.set_gains(kp, ki, kd); }
    void set_torque_constant(double kt) { kt_ = kt; }
    void set_max_torque(double m) {
        max_torque_ = m;
        pid_.set_max_output(m);
    }

    // --- diagnostics -----------------------------------------------------
    double target_torque() const noexcept { return target_; }
    double max_torque()    const noexcept { return max_torque_; }
    double torque_constant() const noexcept { return kt_; }

private:
    PIDController pid_;
    double        kt_          {0.0};   ///< N*m per amp.
    double        max_torque_  {1.0};
    double        target_      {0.0};
};

} // namespace thunders::control
