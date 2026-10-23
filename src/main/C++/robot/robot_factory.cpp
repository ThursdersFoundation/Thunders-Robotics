// =============================================================================
// thunders/robot/robot_factory.cpp
// -----------------------------------------------------------------------------
// Abstract factory for Robot instances.
//
// RobotFactory decouples the construction of a Robot (and its subsystems)
// from the RobotManager that owns the resulting instance. Production code
// uses a DefaultRobotFactory that wires up the real control, navigation,
// and sensor subsystems; tests use a MockRobotFactory that wires up stubs.
//
// Subclasses override build_config() and build_subsystems() to customise
// the resulting Robot.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace thunders::robot {

class Robot;
class RobotConfig;
class ISubsystem;

class RobotFactory {
public:
    RobotFactory() = default;
    virtual ~RobotFactory() = default;

    RobotFactory(const RobotFactory&)            = delete;
    RobotFactory& operator=(const RobotFactory&) = delete;

    /// Builds a fully-wired Robot. Subclasses customise behaviour by
    /// overriding build_config() and build_subsystems().
    std::unique_ptr<Robot> build(const std::string& name);

protected:
    /// Returns the configuration for the new Robot.
    virtual std::shared_ptr<RobotConfig> build_config(const std::string& name) = 0;

    /// Returns the list of subsystems to register on the new Robot.
    virtual std::vector<std::shared_ptr<ISubsystem>> build_subsystems() = 0;
};

/// Default factory used by production code. Wires up the standard
/// control/navigation/sensor subsystems.
class DefaultRobotFactory : public RobotFactory {
public:
    DefaultRobotFactory() = default;

protected:
    std::shared_ptr<RobotConfig> build_config(const std::string& name) override;
    std::vector<std::shared_ptr<ISubsystem>> build_subsystems() override;
};

} // namespace thunders::robot
