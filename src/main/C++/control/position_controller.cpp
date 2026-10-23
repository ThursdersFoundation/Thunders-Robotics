// =============================================================================
// thunders/control/position_controller.cpp
// -----------------------------------------------------------------------------
// Closed-loop position controller wrapping a MotorController.
//
// Uses a profiled PID controller: the setpoint is passed through a
// trapezoidal profile (max velocity, max acceleration) so the motor does not
// lurch. A feedforward term proportional to the profiled velocity provides
// the bulk of the output.
//
// Units: radians for position, rad/s for velocity, rad/s^2 for acceleration.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>

#include "thunders/control/pid_controller.cpp"

namespace thunders::control {

class PositionController {
public:
    PositionController(double kp, double ki, double kd,
                       double max_vel, double max_accel)
        : pid_(kp, ki, kd),
          max_vel_(max_vel),
          max_accel_(max_accel) {
        pid_.set_izone(0.5);
        pid_.set_max_output(12.0);
    }

    /// Sets the goal position. The internal profile generates a feasible
    /// trajectory from the current measured position to the goal.
    void set_goal(double goal_radians, double current_position) {
        goal_ = goal_radians;
        profile_position_ = current_position;
        profile_velocity_ = 0.0;
    }

    /// Advances the profile by dt seconds and computes the motor output.
    double update(double measured_position, double dt_seconds) {
        advance_profile(dt_seconds);
        pid_.set_setpoint(profile_position_);
        const double ff = kv_ * profile_velocity_;
        return ff + pid_.calculate(measured_position);
    }

    bool at_goal(double measured, double tol) const {
        return std::abs(goal_ - measured) < tol
            && std::abs(profile_velocity_) < tol;
    }

    void reset(double current_position) {
        pid_.reset();
        profile_position_ = current_position;
        profile_velocity_ = 0.0;
    }

    // --- tunables --------------------------------------------------------
    void set_gains(double kp, double ki, double kd) { pid_.set_gains(kp, ki, kd); }
    void set_limits(double max_vel, double max_accel) {
        max_vel_ = max_vel;
        max_accel_ = max_accel;
    }
    void set_velocity_feedforward(double kv) { kv_ = kv; }

private:
    /// Trapezoidal profile generator: advances one dt towards the goal.
    void advance_profile(double dt) {
        const double remaining = goal_ - profile_position_;
        const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
        const double distance_to_stop =
            (profile_velocity_ * profile_velocity_) / (2.0 * max_accel_);
        if (std::abs(remaining) <= distance_to_stop) {
            // Decelerate to stop at goal.
            profile_velocity_ -= direction * max_accel_ * dt;
        } else {
            // Accelerate (or hold at max velocity) towards goal.
            profile_velocity_ += direction * max_accel_ * dt;
            profile_velocity_ = std::clamp(profile_velocity_,
                                            -max_vel_, max_vel_);
        }
        profile_position_ += profile_velocity_ * dt;
    }

    PIDController pid_;
    double        max_vel_          {1.0};
    double        max_accel_        {1.0};
    double        kv_               {0.0};
    double        goal_             {0.0};
    double        profile_position_ {0.0};
    double        profile_velocity_ {0.0};
};

} // namespace thunders::control
