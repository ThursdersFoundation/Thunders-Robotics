// =============================================================================
// thunders/navigation/path_planner.cpp
// -----------------------------------------------------------------------------
// Abstract path planner.
//
// A path planner takes a start pose, a goal pose, and an occupancy grid, and
// produces a sequence of waypoints that the path follower can track. The
// default implementation uses A* on the grid; subclasses may use RRT, RRT*,
// or any other planner.
//
// The planner is intentionally synchronous: plan() blocks until the path is
// ready. For long plans, run the planner on a worker thread and feed the
// result to the Navigator via set_path().
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <memory>
#include <vector>

namespace thunders::navigation {

class Waypoint;
class OccupancyGrid;

class PathPlanner {
public:
    PathPlanner() = default;
    virtual ~PathPlanner() = default;

    PathPlanner(const PathPlanner&)            = delete;
    PathPlanner& operator=(const PathPlanner&) = delete;

    /// Plans a path from start to goal on the given grid.
    /// Returns an empty vector if no path exists.
    virtual std::vector<Waypoint> plan(const Waypoint& start,
                                       const Waypoint& goal,
                                       const OccupancyGrid& grid) = 0;

    /// Returns the most recent plan (possibly empty).
    const std::vector<Waypoint>& last_path() const noexcept { return last_path_; }

    /// Returns the wall-clock time spent on the last plan() call, in seconds.
    double last_plan_time_seconds() const noexcept { return last_plan_time_; }

protected:
    std::vector<Waypoint> last_path_;
    double                last_plan_time_ {0.0};
};

/// A* planner on a 2D grid. Uses 8-connected motion with diagonal cost
/// sqrt(2). Heuristic is Euclidean distance.
class AStarPathPlanner : public PathPlanner {
public:
    AStarPathPlanner() = default;

    std::vector<Waypoint> plan(const Waypoint& start,
                               const Waypoint& goal,
                               const OccupancyGrid& grid) override;
};

/// RRT planner. Useful for high-dimensional configuration spaces.
class RRTPathPlanner : public PathPlanner {
public:
    explicit RRTPathPlanner(uint32_t max_iterations = 5000,
                            double   step_size_m    = 0.2)
        : max_iterations_(max_iterations), step_size_m_(step_size_m) {}

    std::vector<Waypoint> plan(const Waypoint& start,
                               const Waypoint& goal,
                               const OccupancyGrid& grid) override;

private:
    uint32_t max_iterations_;
    double   step_size_m_;
};

} // namespace thunders::navigation
