// =============================================================================
// thunders/robot/robot_state.cpp
// -----------------------------------------------------------------------------
// Shared, mutable snapshot of high-level robot inputs.
//
// RobotState is updated by subsystems on every periodic cycle and read by
// commands that need a consistent view of multiple subsystems. The class is
// NOT thread-safe; it is intended to be accessed only from the periodic
// thread. Cross-thread consumers should snapshot the values they need into a
// local copy under a mutex (not provided here).
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>

namespace thunders::robot {

// Forward-declared here to break circular dependencies with robot_mode.cpp.
enum class RobotMode;

class RobotState {
public:
    RobotState() = default;

    // --- mode ------------------------------------------------------------
    RobotMode mode() const noexcept { return mode_; }
    void      set_mode(RobotMode m) noexcept { mode_ = m; }

    // --- timing ----------------------------------------------------------
    double    dt_seconds()     const noexcept { return dt_seconds_; }
    void      set_dt(double dt) noexcept { dt_seconds_ = dt; }

    uint64_t  loop_count()     const noexcept { return loop_count_; }
    void      set_loop_count(uint64_t n) noexcept { loop_count_ = n; }

    // --- pose ( metres / radians ) --------------------------------------
    double    x()  const noexcept { return x_; }
    double    y()  const noexcept { return y_; }
    double    yaw() const noexcept { return yaw_; }
    void      set_pose(double x, double y, double yaw) noexcept {
        x_ = x; y_ = y; yaw_ = yaw;
    }

    // --- velocities ------------------------------------------------------
    double    vx() const noexcept { return vx_; }
    double    vy() const noexcept { return vy_; }
    double    omega() const noexcept { return omega_; }
    void      set_velocity(double vx, double vy, double w) noexcept {
        vx_ = vx; vy_ = vy; omega_ = w;
    }

    // --- safety flags ----------------------------------------------------
    bool      emergency_stop() const noexcept { return emergency_stop_; }
    void      set_emergency_stop(bool v) noexcept { emergency_stop_ = v; }

    bool      brownout() const noexcept { return brownout_; }
    void      set_brownout(bool v) noexcept { brownout_ = v; }

    /// Resets every mutable field to its default. Called by the E-stop
    /// handler so stale state does not bleed into recovery.
    void reset() noexcept {
        dt_seconds_ = 0.0;
        loop_count_ = 0;
        x_ = y_ = yaw_ = 0.0;
        vx_ = vy_ = omega_ = 0.0;
        emergency_stop_ = false;
        brownout_ = false;
    }

private:
    RobotMode mode_              {};
    double    dt_seconds_        {0.0};
    uint64_t  loop_count_        {0};
    double    x_ {0.0}, y_ {0.0}, yaw_ {0.0};
    double    vx_{0.0}, vy_{0.0}, omega_{0.0};
    bool      emergency_stop_    {false};
    bool      brownout_          {false};
};

} // namespace thunders::robot
