// =============================================================================
// thunders/navigation/odometry.cpp
// -----------------------------------------------------------------------------
// Wheel-based odometry for differential and swerve drives.
//
// Odometry integrates wheel encoder readings (and gyro heading, if
// available) into a 2D pose estimate. The estimate drifts over time due to
// wheel slip, encoder quantisation, and integration error; the
// Localization subsystem corrects this drift by fusing the odometry with
// absolute pose observations (e.g., AprilTags).
//
// This file provides a differential-drive odometry implementation. Swerve
// odometry is similar but integrates four modules instead of two; it is
// implemented in a sibling file (not shown here).
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>

namespace thunders::navigation {

class DifferentialDriveOdometry {
public:
    struct Pose {
        double x, y, yaw;
    };

    DifferentialDriveOdometry(double track_width_m,
                              double wheel_radius_m)
        : track_width_m_(track_width_m),
          wheel_radius_m_(wheel_radius_m) {}

    /// Updates the pose estimate given the left/right wheel delta (in
    /// radians) and an optional gyro yaw delta (in radians). When the
    /// gyro delta is provided, it is used in place of the wheel-derived
    /// heading, which dramatically reduces drift.
    Pose update(double left_delta_rad,
                double right_delta_rad,
                double gyro_yaw_delta_rad = 0.0) {
        const double left_dist_m  = left_delta_rad  * wheel_radius_m_;
        const double right_dist_m = right_delta_rad * wheel_radius_m_;
        const double d_dist = 0.5 * (left_dist_m + right_dist_m);

        // Use gyro if provided, else fall back to wheel-derived heading.
        const double d_yaw = (gyro_yaw_delta_rad != 0.0)
            ? gyro_yaw_delta_rad
            : (right_dist_m - left_dist_m) / track_width_m_;

        if (std::abs(d_yaw) < 1e-6) {
            // Straight-line motion.
            pose_.x   += d_dist * std::cos(pose_.yaw);
            pose_.y   += d_dist * std::sin(pose_.yaw);
        } else {
            // Arc motion (exact, no small-angle approximation).
            const double r = d_dist / d_yaw;
            pose_.x   += r * (std::sin(pose_.yaw + d_yaw) - std::sin(pose_.yaw));
            pose_.y   -= r * (std::cos(pose_.yaw + d_yaw) - std::cos(pose_.yaw));
            pose_.yaw += d_yaw;
        }
        // Normalise yaw to (-pi, pi].
        while (pose_.yaw >  M_PI) pose_.yaw -= 2.0 * M_PI;
        while (pose_.yaw <= -M_PI) pose_.yaw += 2.0 * M_PI;
        return pose_;
    }

    void reset_pose(const Pose& p) { pose_ = p; }
    void reset_pose(double x, double y, double yaw) { pose_ = {x, y, yaw}; }

    const Pose& pose() const noexcept { return pose_; }
    double track_width_m() const noexcept { return track_width_m_; }
    double wheel_radius_m() const noexcept { return wheel_radius_m_; }

private:
    double track_width_m_;
    double wheel_radius_m_;
    Pose   pose_ {0.0, 0.0, 0.0};
};

} // namespace thunders::navigation
