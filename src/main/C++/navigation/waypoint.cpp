// =============================================================================
// thunders/navigation/waypoint.cpp
// -----------------------------------------------------------------------------
// A single point along a navigation path.
//
// A Waypoint carries a 2D pose (x, y, yaw) plus optional metadata: a
// tolerance (how close the robot must be to "reach" the waypoint) and a
// speed hint (the maximum speed the robot should travel at while passing
// through this waypoint). Metadata is used by the path follower to slow
// the robot for sharp turns or to stop it at a scoring position.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>

namespace thunders::navigation {

class Waypoint {
public:
    Waypoint() = default;
    Waypoint(double x, double y, double yaw = 0.0)
        : x_(x), y_(y), yaw_(yaw) {}

    // --- accessors -------------------------------------------------------
    double x()   const noexcept { return x_; }
    double y()   const noexcept { return y_; }
    double yaw() const noexcept { return yaw_; }

    void   set_x(double x)   { x_ = x; }
    void   set_y(double y)   { y_ = y; }
    void   set_yaw(double y) { yaw_ = y; }

    double tolerance_m()       const noexcept { return tolerance_m_; }
    void   set_tolerance_m(double t)          { tolerance_m_ = t; }

    double speed_hint_mps()    const noexcept { return speed_hint_mps_; }
    void   set_speed_hint_mps(double s)       { speed_hint_mps_ = s; }

    /// Distance from this waypoint to another, in metres.
    double distance_to(const Waypoint& other) const noexcept {
        const double dx = other.x_ - x_;
        const double dy = other.y_ - y_;
        return std::sqrt(dx * dx + dy * dy);
    }

    /// Angle (yaw) of the line from this waypoint to another, in radians.
    double bearing_to(const Waypoint& other) const noexcept {
        return std::atan2(other.y_ - y_, other.x_ - x_);
    }

    /// True if the given pose is within tolerance_m of this waypoint.
    bool reached(double px, double py) const noexcept {
        const double dx = px - x_;
        const double dy = py - y_;
        return std::sqrt(dx * dx + dy * dy) <= tolerance_m_;
    }

    /// Squared distance (cheaper than distance_to; useful for sorting).
    double distance_to_squared(const Waypoint& other) const noexcept {
        const double dx = other.x_ - x_;
        const double dy = other.y_ - y_;
        return dx * dx + dy * dy;
    }

    bool operator==(const Waypoint& other) const noexcept {
        return x_ == other.x_ && y_ == other.y_ && yaw_ == other.yaw_;
    }
    bool operator!=(const Waypoint& other) const noexcept {
        return !(*this == other);
    }

private:
    double x_                {0.0};
    double y_                {0.0};
    double yaw_              {0.0};
    double tolerance_m_      {0.10};
    double speed_hint_mps_   {1.0};
};

} // namespace thunders::navigation
