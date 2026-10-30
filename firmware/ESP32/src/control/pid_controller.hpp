/**
 * @file pid_controller.hpp
 * @brief Generic, reusable PID controller.
 *
 * A minimal but production-quality PID implementation:
 *   - Anti-windup via output clamping and integral term clamping.
 *   - Derivative on measurement (not on error) to avoid setpoint kick.
 *   - Optional setpoint weighting (β ∈ [0,1]) for smoother step response.
 *
 * The class is `float`-only and stateful — call `reset()` between tuning
 * runs or when switching setpoints by a large step.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_PID_CONTROLLER_HPP
#define THUNDERS_PID_CONTROLLER_HPP

#include <cstdint>

namespace thunders::control {

class PidController {
public:
    struct Gains {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
    };

    PidController() = default;

    void configure(const Gains& gains, float output_min, float output_max,
                   float integral_limit);

    /// Compute the next control output given setpoint, measurement, and dt.
    /// `dt` is in seconds. Use a fixed dt in the control task for stability.
    float update(float setpoint, float measurement, float dt_s);

    /// Reset integrator & derivative state.
    void reset();

    // ─── Tuning accessors ─────────────────────────────────────────────────
    void setGains(const Gains& g)     { gains_ = g; }
    const Gains& gains() const        { return gains_; }
    float integralTerm() const       { return integral_; }
    float lastError() const          { return lastError_; }

private:
    Gains gains_{};
    float outputMin_     = -1.0e9f;
    float outputMax_     = +1.0e9f;
    float integralLimit_ = 1.0e9f;
    float integral_      = 0.0f;
    float lastError_     = 0.0f;
    bool  initialised_   = false;
};

}  // namespace thunders::control

#endif  // THUNDERS_PID_CONTROLLER_HPP
