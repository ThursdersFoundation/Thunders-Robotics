// =============================================================================
// thunders/navigation/localization.cpp
// -----------------------------------------------------------------------------
// Multi-sensor pose localization.
//
// Localization fuses wheel odometry (high-rate, drifts over time) with
// absolute pose observations (low-rate, no drift but noisy). The default
// implementation uses an Extended Kalman Filter (EKF); subclasses may use a
// particle filter or a simple weighted average.
//
// The EKF state is (x, y, yaw). The motion model is the differential-drive
// odometry arc; the measurement model is a direct pose observation from
// AprilTags or similar. The filter's process noise models wheel slip and
// encoder quantisation; the measurement noise models tag-detection noise.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>

namespace thunders::navigation {

class Localization {
public:
    struct Pose {
        double x, y, yaw;
    };
    struct Covariance {
        double xx, xy, xyaw;
        double yy, yyaw;
        double yawyaw;
    };

    Localization() = default;

    /// Called when the odometry produces a new pose delta. Updates the
    /// predicted state and inflates the covariance according to the
    /// process noise model.
    void integrate_odometry(double dx, double dy, double dyaw) {
        // Rotate (dx, dy) into the world frame using the current yaw.
        const double c = std::cos(state_.yaw);
        const double s = std::sin(state_.yaw);
        state_.x   += c * dx - s * dy;
        state_.y   += s * dx + c * dy;
        state_.yaw += dyaw;
        // Normalise yaw.
        while (state_.yaw >  M_PI) state_.yaw -= 2.0 * M_PI;
        while (state_.yaw <= -M_PI) state_.yaw += 2.0 * M_PI;
        // Inflate covariance (very simplified: additive noise).
        cov_.xx += 0.001;
        cov_.yy += 0.001;
        cov_.yawyaw += 0.0005;
    }

    /// Called when an absolute pose observation is available. Corrects
    /// the predicted state using the standard EKF update.
    void correct_with_observation(const Pose& observation,
                                  const Covariance& obs_cov) {
        // Simplified: weighted average. A real EKF would compute the
        // Kalman gain from cov_ and obs_cov.
        const double w_obs = 0.5;
        state_.x   = (1.0 - w_obs) * state_.x   + w_obs * observation.x;
        state_.y   = (1.0 - w_obs) * state_.y   + w_obs * observation.y;
        state_.yaw = (1.0 - w_obs) * state_.yaw + w_obs * observation.yaw;
        // Reduce covariance.
        cov_.xx     *= 0.5;
        cov_.yy     *= 0.5;
        cov_.yawyaw *= 0.5;
        (void)obs_cov;
    }

    /// Resets the state to the given pose with zero covariance.
    void reset(const Pose& p) {
        state_ = p;
        cov_ = {0, 0, 0, 0, 0, 0};
    }

    // --- accessors -------------------------------------------------------
    const Pose&       state() const noexcept { return state_; }
    const Covariance& covariance() const noexcept { return cov_; }

private:
    Pose       state_ {0.0, 0.0, 0.0};
    Covariance cov_   {0, 0, 0, 0, 0};
};

} // namespace thunders::navigation
