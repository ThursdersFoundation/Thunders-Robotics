// =============================================================================
// thunders/navigation/navigator.cpp
// -----------------------------------------------------------------------------
// Top-level navigation coordinator.
//
// The Navigator ties together the PathPlanner, PathFollower, and
// Localization subsystems. A typical navigation cycle looks like:
//
//   1. set_goal(pose)  - the planner generates a path from the current pose
//                        to the goal.
//   2. update(dt)      - the follower computes the next commanded velocity
//                        using the current pose and the path; the
//                        localization subsystem fuses sensor data to update
//                        the pose estimate.
//   3. at_goal()       - true once the robot is within tolerance of the goal.
//
// The Navigator is mode-aware: it refuses to issue commands when the robot
// is in DISABLED or EMERGENCY mode.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <memory>

namespace thunders::navigation {

class PathPlanner;
class PathFollower;
class Localization;
class Waypoint;

class Navigator {
public:
    struct Pose {
        double x, y, yaw;
    };
    struct Velocity {
        double vx, vy, omega;
    };

    Navigator(std::shared_ptr<PathPlanner>   planner,
              std::shared_ptr<PathFollower>  follower,
              std::shared_ptr<Localization>  localization);

    /// Sets a new navigation goal. Triggers a replan on the next update().
    void set_goal(const Pose& goal);

    /// Advances the navigation by dt seconds. Returns the commanded
    /// velocity to be sent to the drivetrain.
    Velocity update(double dt_seconds);

    /// True when the robot is within tolerance of the current goal.
    bool at_goal() const noexcept;

    /// Cancels the current navigation goal. The robot will decelerate to
    /// a stop on the next update().
    void cancel();

    // --- accessors -------------------------------------------------------
    std::shared_ptr<PathPlanner>   planner()      const noexcept { return planner_; }
    std::shared_ptr<PathFollower>  follower()     const noexcept { return follower_; }
    std::shared_ptr<Localization>  localization() const noexcept { return localization_; }

private:
    std::shared_ptr<PathPlanner>   planner_;
    std::shared_ptr<PathFollower>  follower_;
    std::shared_ptr<Localization>  localization_;
    Pose                           goal_           {0, 0, 0};
    bool                           has_goal_       {false};
    bool                           cancelled_      {false};
    double                         goal_tolerance_ {0.05};
};

} // namespace thunders::navigation
