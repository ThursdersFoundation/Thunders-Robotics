// =============================================================================
// thunders/control/servo_controller.cpp
// -----------------------------------------------------------------------------
// Controller for hobby-style PWM servos (e.g., HS-645MG, HS-805BB).
//
// Wraps a PWM output and exposes a degrees-based API. The servo's pulse
// width range and travel range are configurable so the same controller can
// drive servos of different makes without code changes.
//
// Calibration procedure:
//   1. Set min_pulse_us and max_pulse_us to the servo's documented range.
//   2. Command 0 degrees and verify the servo is at its mechanical min.
//   3. Command max_travel_degrees and verify the servo is at its mechanical
//      max.
//   4. Adjust offsets until both endpoints are correct.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <algorithm>
#include <cstdint>

namespace thunders::control {

class ServoController {
public:
    ServoController(uint32_t pwm_channel,
                    double min_pulse_us,
                    double max_pulse_us,
                    double max_travel_degrees)
        : pwm_channel_(pwm_channel),
          min_pulse_us_(min_pulse_us),
          max_pulse_us_(max_pulse_us),
          max_travel_degrees_(max_travel_degrees) {}

    /// Commands the servo to the given angle, clamped to [0, max_travel].
    void set_angle_degrees(double angle) {
        const double clamped = std::clamp(angle, 0.0, max_travel_degrees_);
        const double fraction = clamped / max_travel_degrees_;
        const double pulse_us = min_pulse_us_
            + fraction * (max_pulse_us_ - min_pulse_us_);
        last_pulse_us_ = pulse_us;
        // Real implementation would call hal::set_pwm_pulse(pwm_channel_, pulse_us).
    }

    /// Convenience: sets the angle as a fraction of max travel in [0, 1].
    void set_fraction(double f) {
        set_angle_degrees(std::clamp(f, 0.0, 1.0) * max_travel_degrees_);
    }

    /// Disables the PWM output; the servo will go limp.
    void disable() {
        last_pulse_us_ = 0.0;
        // Real implementation would call hal::disable_pwm(pwm_channel_).
    }

    // --- accessors -------------------------------------------------------
    uint32_t pwm_channel()        const noexcept { return pwm_channel_; }
    double   min_pulse_us()       const noexcept { return min_pulse_us_; }
    double   max_pulse_us()       const noexcept { return max_pulse_us_; }
    double   max_travel_degrees() const noexcept { return max_travel_degrees_; }
    double   last_pulse_us()      const noexcept { return last_pulse_us_; }

    /// Returns the last commanded angle, derived from the last pulse width.
    double last_angle_degrees() const noexcept {
        if (last_pulse_us_ <= 0.0) return 0.0;
        const double f = (last_pulse_us_ - min_pulse_us_)
                       / (max_pulse_us_ - min_pulse_us_);
        return std::clamp(f, 0.0, 1.0) * max_travel_degrees_;
    }

private:
    uint32_t pwm_channel_;
    double   min_pulse_us_;
    double   max_pulse_us_;
    double   max_travel_degrees_;
    double   last_pulse_us_ {0.0};
};

} // namespace thunders::control
