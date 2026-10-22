// =============================================================================
// thunders/sensors/sensor_manager.cpp
// -----------------------------------------------------------------------------
// Lifecycle manager for a collection of sensors.
//
// SensorManager owns every Sensor in the robot and provides:
//   - init_all(): calls init() on every sensor, returns the list of failures.
//   - read_all(): calls read() on every sensor, returns the list of failures.
//   - close_all(): closes every sensor.
//   - health_report(): returns a per-sensor health summary.
//
// The manager is thread-safe. Sensors may be added or removed at runtime;
// reads are serialised through an internal mutex.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace thunders::sensors {

class Sensor;

class SensorManager {
public:
    SensorManager() = default;
    ~SensorManager() = default;

    SensorManager(const SensorManager&)            = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    /// Registers a sensor under the given name. Throws if the name is
    /// already in use.
    void add(const std::string& name, std::shared_ptr<Sensor> sensor);

    /// Removes a previously-registered sensor. Closes it first.
    void remove(const std::string& name);

    /// Returns the sensor with the given name, or nullptr if not found.
    std::shared_ptr<Sensor> get(const std::string& name) const;

    /// Calls init() on every registered sensor. Returns the names of
    /// sensors whose init() returned false.
    std::vector<std::string> init_all();

    /// Calls read() on every registered sensor. Returns the names of
    /// sensors whose read() returned false.
    std::vector<std::string> read_all();

    /// Calls close() on every registered sensor.
    void close_all();

    /// Returns a per-sensor health report. Each entry is (name, healthy).
    std::vector<std::pair<std::string, bool>> health_report() const;

    /// Returns the number of registered sensors.
    std::size_t size() const;

private:
    mutable std::mutex                                       mutex_;
    std::unordered_map<std::string, std::shared_ptr<Sensor>> sensors_;
};

} // namespace thunders::sensors
