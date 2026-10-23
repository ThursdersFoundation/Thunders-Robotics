// =============================================================================
// thunders/control/differential_drive.cpp
// -----------------------------------------------------------------------------
// Kinematics and odometry for a 2-wheel differential (tank) drive.
//
// A differential drive robot has two parallel driven wheels; the robot's
// motion is controlled by varying the relative speed of the wheels. This
// class provides:
//
//   - inverse_kinematics(): given a desired linear+angular velocity,
//     returns the per-wheel velocities.
//   - forward_kinematics(): given the per-wheel measured velocities,
//     returns the resulting linear+angular velocity.
//   - update_odometry(): integrates the wheel velocities into a pose
//     (x, y, yaw) over time.
//
// Units: metres, radians, seconds.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma "once"

#include <cmath>

namespace thunders::control {

class DifferentialDrive {
public:
    DifferentialDrive(double wheel_radius_m, double track_width_m)
        : wheel_radius_m_(wheel_radius_m),
          track_width_m_(track_width_m) {}

    struct ChassisSpeeds {
        double linear_vx;   ///< m/s, forward
        double angular_vz;  ///< rad/s, CCW positive
    };

    struct WheelSpeeds {
        double left_rad_per_s;
        double right_rad_per_s;
    };

    struct Pose {
        double x;
        double y;
        double yaw;
    };

    /// Inverse kinematics: chassis speeds -> wheel angular velocities.
    WheelSpeeds inverse_kinematics(const ChassisSpeeds& cs) const {
        const double v = cs.linear_vx;
        const double w = cs.angular_vz;
        const double wheel_v_left  = v - 0.5 * w * track_width_m_;
        const double wheel_v_right = v + 0.5 * w * track_width_m_;
        return { wheel_v_left  / wheel_radius_m_,
                 wheel_v_right / wheel_radius_m_ };
    }

    /// Forward kinematics: wheel angular velocities -> chassis speeds.
    ChassisSpeeds forward_kinematics(const WheelSpeeds& ws) const {
        const double v_left  = ws.left_rad_per_s  * wheel_radius_m_;
        const double v_right = ws.right_rad_per_s * wheel_radius_m_;
        const double v = 0.5 * (v_left + v_right);
        const double w = (v_right - v_left) / track_width_m_;
        return { v, w };
    }

    /// Integrates wheel motions over dt to update the robot's pose.
    /// Uses the exact arc-length formulation (no small-angle approximation).
    void update_odometry(const WheelSpeeds& measured, double dt_seconds) {
        const auto cs = forward_kinematics(measured);
        const double v = cs.linear_vx;
        const double w = cs.angular_vz;
        if (std::abs(w) < 1e-6) {
            // Straight-line motion.
            pose_.x     += v * dt_seconds * std::cos(pose_.yaw);
            pose_.y     += v * dt_seconds * std::sin(pose_.yaw);
        } else {
            // Circular arc.
            const double dtheta = w * dt_seconds;
            const double r = v / w;
            pose_.x     += r * (std::sin(pose_.yaw + dtheta) - std::sin(pose_.yaw));
            pose_.y     -= r * (std::cos(pose_.yaw + dtheta) - std::cos(pose_.yaw));
            pose_.yaw   += dtheta;
        }
        // Normalise yaw to (-pi, pi].
        while (pose_.yaw >  M_PI) pose_.yaw -= 2.0 * M_PI;
        while (pose_.yaw <= -M_PI) pose_.yaw += 2.0 * M_PI;
    }

    /// Resets the pose to the given (x, y, yaw). Use when a known starting
    /// position is available (e.g., from a vision target).
    void reset_pose(double x, double y, double yaw) {
        pose_ = { x, y, yaw };
    }

    // --- accessors -------------------------------------------------------
    const Pose& pose()           const noexcept { return pose_; }
    double      wheel_radius_m() const noexcept { return wheel_radius_m_; }
    double      track_width_m()  const noexcept { return track_width_m_; }

private:
    double wheel_radius_m_;
    double track_width_m_;
    Pose   pose_ {0.0, 0.0, 0.0};
    };
};
