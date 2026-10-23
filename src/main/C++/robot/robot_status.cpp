// =============================================================================
// thunders/robot/robot_status.cpp
// -----------------------------------------------------------------------------
// Diagnostic snapshot of the robot's high-level health.
//
// RobotStatus is a write-once-per-tick record of things the drive team wants
// to see at a glance: current phase, last loop overrun, last mode change,
// and a rolling fault counter. It is intentionally distinct from RobotState
// (which holds values consumed by commands); RobotStatus is purely
// informational.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <chrono>
#include <cstdint>

namespace thunders::robot {

enum class RobotMode;

class RobotStatus {
public:
    enum class Phase {
        UNINITIALISED,
        INITIALISING,
        READY,
        RUNNING,
        DEGRADED,
        FAULTED,
        STOPPED
    };

    RobotStatus() = default;

    // --- phase -----------------------------------------------------------
    Phase   phase()                    const noexcept { return phase_; }
    void    set_phase(Phase p)                  noexcept { phase_ = p; }

    // --- loop health -----------------------------------------------------
    bool    loop_overrun()            const noexcept { return loop_overrun_; }
    void    set_loop_overrun(bool v)           noexcept {
        loop_overrun_ = v;
        if (v) ++overrun_count_;
    }

    uint32_t overrun_count()          const noexcept { return overrun_count_; }

    // --- mode change bookkeeping ----------------------------------------
    void    record_mode_change(RobotMode /*new_mode*/) {
        ++mode_change_count_;
        last_mode_change_at_ = std::chrono::steady_clock::now();
    }
    uint32_t mode_change_count()      const noexcept { return mode_change_count_; }

    // --- fault counter ---------------------------------------------------
    uint32_t fault_count()            const noexcept { return fault_count_; }
    void     record_fault()                    noexcept { ++fault_count_; }

    /// Resets every counter except phase. Called at the start of a match.
    void reset_counters() noexcept {
        overrun_count_ = 0;
        mode_change_count_ = 0;
        fault_count_ = 0;
    }

private:
    Phase      phase_              {Phase::UNINITIALISED};
    bool       loop_overrun_       {false};
    uint32_t   overrun_count_      {0};
    uint32_t   mode_change_count_  {0};
    uint32_t   fault_count_        {0};
    std::chrono::steady_clock::time_point last_mode_change_at_;
};

} // namespace thunders::robot
