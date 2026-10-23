// =============================================================================
// thunders/robot/robot_config.cpp
// -----------------------------------------------------------------------------
// Strongly-typed runtime configuration for a Robot instance.
//
// RobotConfig is a plain data carrier (POD-like) used to parameterise a
// Robot at construction time. It is intentionally a value type: copying is
// cheap, and there is no behaviour beyond the accessors. Validation lives in
// the constructor so that an invalid config fails fast at startup rather than
// producing surprising behaviour deep in a periodic loop.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <stdexcept>
#include <string>

namespace thunders::robot {

class RobotConfig {
public:
    RobotConfig(std::string name,
                std::string version,
                double      loop_period_seconds,
                double      max_loop_dt_seconds,
                bool        enable_watchdog)
        : name_(std::move(name)),
          version_(std::move(version)),
          loop_period_seconds_(loop_period_seconds),
          max_loop_dt_seconds_(max_loop_dt_seconds),
          enable_watchdog_(enable_watchdog) {
        if (loop_period_seconds_ <= 0.0) {
            throw std::invalid_argument("RobotConfig: loop_period must be > 0");
        }
        if (max_loop_dt_seconds_ <= loop_period_seconds_) {
            throw std::invalid_argument(
                "RobotConfig: max_loop_dt must be > loop_period");
        }
        if (name_.empty()) {
            throw std::invalid_argument("RobotConfig: name must not be empty");
        }
    }

    // --- accessors -------------------------------------------------------
    const std::string& name()                  const noexcept { return name_; }
    const std::string& version()               const noexcept { return version_; }
    double             loop_period_seconds()   const noexcept { return loop_period_seconds_; }
    double             max_loop_dt_seconds()   const noexcept { return max_loop_dt_seconds_; }
    bool               enable_watchdog()       const noexcept { return enable_watchdog_; }

private:
    std::string name_;
    std::string version_;
    double      loop_period_seconds_;
    double      max_loop_dt_seconds_;
    bool        enable_watchdog_;
};

} // namespace thunders::robot
