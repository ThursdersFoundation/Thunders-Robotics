// =============================================================================
// thunders/control/pid_controller.cpp
// -----------------------------------------------------------------------------
// Generic PID controller with anti-windup and derivative filtering.
//
// This is the standard PID implementation used throughout the library. It
// adds two features commonly needed on real robots:
//
//   1. Integral clamping (iZone) — the integral term is clamped to a
//      configurable range to prevent wind-up when the actuator saturates.
//   2. Derivative filtering — the derivative term is passed through a
//      first-order low-pass filter to suppress noise.
//
// Tuning procedure (standard):
//   1. Set KI = KD = 0; increase KP until the system oscillates.
//   2. Halve KP.
//   3. Increase KD until oscillation damps within one or two cycles.
//   4. Increase KI until steady-state error is removed.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>

namespace thunders::control {

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    /// Computes the controller output for the given measurement and the
    /// currently-set setpoint. Returns a value clamped to
    /// [-max_output_, +max_output_].
    double calculate(double measurement) {
        const double error = setpoint_ - measurement;

        // Integral term with anti-windup.
        if (std::abs(error) < i_zone_ || i_zone_ <= 0.0) {
            integral_ += error;
        } else {
            integral_ = 0.0;
        }
        integral_ = std::clamp(integral_, -i_zone_, i_zone_);

        // Derivative term with low-pass filter.
        const double raw_d = error - previous_error_;
        filtered_d_ = filtered_d_ + alpha_ * (raw_d - filtered_d_);
        previous_error_ = error;

        const double out = kp_ * error + ki_ * integral_ + kd_ * filtered_d_;
        return std::clamp(out, -max_output_, max_output_);
    }

    // --- configuration ---------------------------------------------------
    void set_gains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }
    void set_setpoint(double sp)               { setpoint_ = sp; }
    void set_izone(double z)                   { i_zone_ = std::abs(z); }
    void set_max_output(double m)              { max_output_ = std::abs(m); }
    void set_derivative_filter(double a)       { alpha_ = std::clamp(a, 0.0, 1.0); }

    // --- state -----------------------------------------------------------
    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
        filtered_d_ = 0.0;
    }

    bool at_setpoint(double measurement, double tol) const {
        return std::abs(setpoint_ - measurement) < tol;
    }

    // --- accessors (used by tests and dashboards) ------------------------
    double kp() const noexcept { return kp_; }
    double ki() const noexcept { return ki_; }
    double kd() const noexcept { return kd_; }
    double setpoint() const noexcept { return setpoint_; }
    double integral() const noexcept { return integral_; }

private:
    double kp_           {0.0};
    double ki_           {0.0};
    double kd_           {0.0};
    double setpoint_     {0.0};
    double i_zone_       {1.0};
    double max_output_   {1.0};
    double alpha_        {0.5};
    double integral_     {0.0};
    double previous_error_{0.0};
    double filtered_d_   {0.0};
};

} // namespace thunders::control
