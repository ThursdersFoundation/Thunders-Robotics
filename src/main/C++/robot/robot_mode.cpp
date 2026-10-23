// =============================================================================
// thunders/robot/robot_mode.cpp
// -----------------------------------------------------------------------------
// Strongly-typed enumeration of robot operating modes.
//
// RobotMode is used throughout the library to gate behaviour: autonomous
// commands only run in AUTONOMOUS, teleop commands only in TELEOP, and
// safety-critical subsystems refuse to move in DISABLED.
//
// The enum is intentionally a scoped enum (enum class) so that mode values
// do not pollute the surrounding namespace and cannot be implicitly
// converted to integers.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>
#include <string_view>

namespace thunders::robot {

/// High-level operating mode reported by the driver station.
enum class RobotMode : uint8_t {
    DISABLED    = 0,  ///< Motors disabled; commands cannot run.
    IDLE        = 1,  ///< Initialised but no mode active.
    AUTONOMOUS  = 2,  ///< Running an autonomous routine.
    TELEOP      = 3,  ///< Driver-controlled.
    TEST        = 4,  ///< Diagnostic / test mode.
    EMERGENCY   = 5,  ///< E-stop latched; requires explicit recovery.
};

/// Returns a short, lowercase name for the given mode. Used in logs and
/// dashboard telemetry. Returns "unknown" if the mode is not recognised.
inline constexpr std::string_view mode_name(RobotMode m) noexcept {
    switch (m) {
        case RobotMode::DISABLED:   return "disabled";
        case RobotMode::IDLE:       return "idle";
        case RobotMode::AUTONOMOUS: return "autonomous";
        case RobotMode::TELEOP:     return "teleop";
        case RobotMode::TEST:       return "test";
        case RobotMode::EMERGENCY:  return "emergency";
    }
    return "unknown";
}

/// True if the mode allows commanded motion. DISABLED and EMERGENCY do not.
inline constexpr bool mode_allows_motion(RobotMode m) noexcept {
    return m == RobotMode::AUTONOMOUS
        || m == RobotMode::TELEOP
        || m == RobotMode::TEST;
}

/// True if the mode allows operator input. Only TELEOP and TEST do.
inline constexpr bool mode_allows_operator_input(RobotMode m) noexcept {
    return m == RobotMode::TELEOP || m == RobotMode::TEST;
}

} // namespace thunders::robot
