// =============================================================================
// thunders/navigation/path_follower.cpp
// -----------------------------------------------------------------------------
// Pure-pursuit path follower.
//
// Pure pursuit is a simple, robust path-tracking algorithm. The robot looks
// ahead a fixed distance along the path, computes the curvature required to
// reach that look-ahead point, and steers along that curvature. The
// look-ahead distance trades off stability (large) against responsiveness
// (small); a typical starting value is 1 m.
//
// Outputs are commanded linear and angular velocities in m/s and rad/s.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>
#include <vector>

namespace thunders::navigation {

class Waypoint;

class PathFollower {
public:
    PathFollower(double lookahead_m, double max_linear_v, double max_angular_v)
        : lookahead_m_(lookahead_m),
          max_linear_v_(max_linear_v),
          max_angular_v_(max_angular_v) {}

    struct Pose {
        double x, y, yaw;
    };
    struct Command {
        double linear_vx;   ///< m/s
        double angular_vz;  ///< rad/s
    };

    /// Sets the path to follow. Resets the progress index to zero.
    void set_path(const std::vector<Waypoint>& path);

    /// Computes the next commanded velocity given the current pose.
    /// Returns zero velocity when the end of the path is reached.
    Command update(const Pose& current);

    /// True when the robot has reached the final waypoint.
    bool at_end() const noexcept;

    /// Resets the follower to the start of the current path.
    void reset();

    // --- tunables --------------------------------------------------------
    void set_lookahead(double m)   { lookahead_m_ = m; }
    void set_max_linear_v(double v){ max_linear_v_ = v; }
    void set_max_angular_v(double v){ max_angular_v_ = v; }

    // --- diagnostics -----------------------------------------------------
    double progress_fraction() const noexcept;
    size_t current_index()     const noexcept { return idx_; }

private:
    std::vector<Waypoint> path_;
    size_t                idx_            {0};
    double                lookahead_m_    {1.0};
    double                max_linear_v_   {1.0};
    double                max_angular_v_  {1.0};
    bool                  at_end_         {false};
};

} // namespace thunders::navigation
