// =============================================================================
// thunders/robot/robot_manager.cpp
// -----------------------------------------------------------------------------
// Lifetime manager for one or more Robot instances.
//
// RobotManager owns the Robot instances created by RobotFactory and is
// responsible for starting, stopping, and querying them. In a typical
// deployment there is exactly one Robot and one RobotManager; the manager
// exists to support multi-robot test benches and simulation swarms.
//
// The manager is thread-safe: start/stop/query may be called from any
// thread. Internally it serialises access through a mutex.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace thunders::robot {

class Robot;
class RobotFactory;

class RobotManager {
public:
    RobotManager() = default;
    ~RobotManager() = default;

    RobotManager(const RobotManager&)            = delete;
    RobotManager& operator=(const RobotManager&) = delete;

    /// Creates and registers a Robot with the given name. Throws if the
    /// name is already in use. Returns a shared_ptr so the caller can
    /// retain a handle.
    std::shared_ptr<Robot> create(const std::string& name,
                                  std::shared_ptr<RobotFactory> factory);

    /// Looks up a previously-created Robot by name. Returns nullptr if no
    /// such Robot exists.
    std::shared_ptr<Robot> get(const std::string& name) const;

    /// Starts the named Robot (calls Robot::init). Throws if not found.
    void start(const std::string& name);

    /// Stops the named Robot (calls Robot::stop). Throws if not found.
    void stop(const std::string& name);

    /// Stops every registered Robot. Called on shutdown.
    void stop_all();

    /// Returns the number of currently-registered Robots.
    std::size_t size() const;

private:
    struct Entry {
        std::shared_ptr<Robot> robot;
        bool                   started {false};
    };

    mutable std::mutex                              mutex_;
    std::unordered_map<std::string, Entry>          robots_;
};

} // namespace thunders::robot
