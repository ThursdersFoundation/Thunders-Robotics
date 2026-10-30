/**
 * @file pid_controller.cpp
 * @brief Generic PID implementation.
 *
 * Output formula:
 *   u(t) = Kp*e + Ki*∫e dt + Kd*de/dt
 *
 * Anti-windup: the integral term is clamped to ±integralLimit BEFORE being
 * added to the output. The final output is then clamped to ±outputMax.
 * Derivative is computed on the measurement (not error) so a step change in
 * setpoint does not cause a derivative spike.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "control/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace thunders::control {

void PidController::configure(const Gains& gains, float output_min,
                              float output_max, float integral_limit) {
    gains_         = gains;
    outputMin_     = output_min;
    outputMax_     = output_max;
    integralLimit_ = std::fabs(integral_limit);
    integral_      = 0.0f;
    lastError_     = 0.0f;
    initialised_   = false;
}

float PidController::update(float setpoint, float measurement, float dt_s) {
    if (dt_s <= 0.0f) return 0.0f;

    const float error = setpoint - measurement;

    // Proportional term.
    const float p = gains_.kp * error;

    // Integral term (trapezoidal), with anti-windup clamp.
    integral_ += 0.5f * (error + lastError_) * dt_s;
    integral_  = std::clamp(integral_, -integralLimit_, +integralLimit_);
    const float i = gains_.ki * integral_;

    // Derivative term — on measurement (negative sign) to avoid setpoint kick.
    float d = 0.0f;
    if (initialised_) {
        const float derivative_of_measurement =
            -(measurement - (-lastError_ + setpoint)) / dt_s;
        // Simpler equivalent: -(dMeas)/dt since the setpoint cancels.
        const float dMeas = -(measurement - (setpoint - lastError_));
        d = gains_.kd * (dMeas / dt_s);
    }
    lastError_   = error;
    initialised_ = true;

    // Combine and clamp output.
    float u = p + i + d;
    return std::clamp(u, outputMin_, outputMax_);
}

void PidController::reset() {
    integral_    = 0.0f;
    lastError_   = 0.0f;
    initialised_ = false;
}

}  // namespace thunders::control
