// =============================================================================
// thunders/robot/robot.cpp
// -----------------------------------------------------------------------------
// Top-level Robot entity for the Thunders robotics library.
//
// The Robot class aggregates every subsystem (control, navigation, sensors),
// holds a shared RobotState, and dispatches lifecycle events (init / periodic
// / stop) to every owned component. Subsystem classes never reference each
// other directly; they go through the Robot (or through the RobotManager) so
// the dependency graph stays explicit and testable.
//
// Sibling types (RobotConfig, RobotState, RobotStatus, RobotMode) are
// forward-declared here; their full definitions live in the matching .cpp
// files in this directory. The build system compiles every .cpp in this
// folder into a single static library, so all definitions are available at
// link time.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

namespace thunders::robot {

// --- Forward declarations (defined in sibling .cpp files) -------------------
class RobotConfig;
class RobotState;
class RobotStatus;
enum class RobotMode;

/// Interface implemented by every subsystem owned by a Robot.
class ISubsystem {
public:
    virtual ~ISubsystem() = default;
    virtual void init() {}
    virtual void periodic(double /*dt_seconds*/) {}
    virtual void stop() {}
    virtual const char* name() const noexcept = 0;
};

/// Top-level Robot entity. Owns subsystems and dispatches lifecycle events.
class Robot {
public:
    explicit Robot(std::shared_ptr<RobotConfig> config);
    ~Robot();

    Robot(const Robot&)            = delete;
    Robot& operator=(const Robot&) = delete;

    /// One-time initialisation. Idempotent.
    void init();

    /// Called once per control loop. Computes dt and forwards to subsystems.
    void periodic();

    /// Stops every subsystem and transitions to DISABLED.
    void stop();

    /// Transitions to a new operating mode.
    void set_mode(RobotMode mode);

    /// Registers a subsystem that will receive lifecycle callbacks.
    void register_subsystem(std::shared_ptr<ISubsystem> sub);

    // --- accessors -------------------------------------------------------
    std::shared_ptr<RobotState>  state()  const noexcept { return state_; }
    std::shared_ptr<RobotStatus> status() const noexcept { return status_; }
    std::shared_ptr<RobotConfig> config() const noexcept { return config_; }
    double                       uptime_seconds() const;

private:
    std::shared_ptr<RobotConfig>                  config_;
    std::shared_ptr<RobotState>                   state_;
    std::shared_ptr<RobotStatus>                  status_;
    std::vector<std::shared_ptr<ISubsystem>>      subsystems_;
    RobotMode                                     mode_;
    std::chrono::steady_clock::time_point         started_at_;
    std::chrono::steady_clock::time_point         last_periodic_at_;
    uint64_t                                      loop_count_;
};

} // namespace thunders::robot
