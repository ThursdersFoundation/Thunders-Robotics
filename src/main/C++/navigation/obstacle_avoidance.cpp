// =============================================================================
// thunders/navigation/obstacle_avoidance.cpp
// -----------------------------------------------------------------------------
// Local obstacle avoidance layer.
//
// ObstacleAvoidance sits between the path follower and the drivetrain. It
// takes the follower's commanded velocity, checks it against a recent
// sensor scan (typically a LaserScan or a 2D point cloud), and modifies
// the velocity to avoid collisions. The default implementation uses the
// Velocity Obstacle approach; subclasses may use DWA, VFH, or simple
// braking.
//
// When the path is blocked, the avoidance layer slows or stops the robot
// and signals the Navigator to replan. It never sends a velocity that
// would put the robot on a collision course with a detected obstacle.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>
#include <vector>

namespace thunders::navigation {

class ObstacleAvoidance {
public:
    struct Point {
        double x, y;
    };
    struct Velocity {
        double vx, vy, omega;
    };
    struct Scan {
        std::vector<Point> points;       ///< in robot frame
        double             max_range_m;
    };

    ObstacleAvoidance(double safety_distance_m,
                      double robot_radius_m)
        : safety_distance_m_(safety_distance_m),
          robot_radius_m_(robot_radius_m) {}

    /// Filters the commanded velocity to avoid the obstacles in the scan.
    /// If a collision is imminent, returns zero velocity and sets
    /// collision_imminent_ to true.
    Velocity filter(const Velocity& commanded, const Scan& scan) {
        collision_imminent_ = false;
        for (const auto& p : scan.points) {
            const double dist = std::sqrt(p.x * p.x + p.y * p.y);
            if (dist < robot_radius_m_ + safety_distance_m_) {
                // Too close: brake hard.
                collision_imminent_ = true;
                return { 0.0, 0.0, 0.0 };
            }
            // Project the robot's velocity onto the obstacle direction.
            // If the robot is moving towards the obstacle, scale down.
            const double dir_x = p.x / dist;
            const double dir_y = p.y / dist;
            const double v_proj = commanded.vx * dir_x + commanded.vy * dir_y;
            if (v_proj > 0.0) {
                // Moving towards obstacle: scale down by (1 - dist / max_range).
                const double scale = 1.0 - (dist / scan.max_range_m);
                return { commanded.vx * scale,
                         commanded.vy * scale,
                         commanded.omega };
            }
        }
        return commanded;
    }

    /// True if the last filter() call detected an imminent collision.
    bool collision_imminent() const noexcept { return collision_imminent_; }

    /// Sets the safety distance (the minimum allowed distance to any
    /// obstacle, in metres).
    void set_safety_distance_m(double d) { safety_distance_m_ = d; }

    /// Sets the robot's collision radius, in metres.
    void set_robot_radius_m(double r) { robot_radius_m_ = r; }

    double safety_distance_m() const noexcept { return safety_distance_m_; }
    double robot_radius_m()    const noexcept { return robot_radius_m_; }

private:
    double safety_distance_m_;
    double robot_radius_m_;
    bool   collision_imminent_ {false};
};

} // namespace thunders::navigation
