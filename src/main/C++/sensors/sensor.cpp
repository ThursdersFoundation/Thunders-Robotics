// =============================================================================
// thunders/sensors/sensor.cpp
// -----------------------------------------------------------------------------
// Abstract base class for all sensors.
//
// The Sensor interface normalises the lifecycle of every sensor in the
// Thunders library: init() opens the device and configures sample rate,
// read() pulls the latest sample (non-blocking), close() releases the
// device. The interface is intentionally synchronous; sensors that need
// background polling wrap themselves in a worker thread internally.
//
// Every sensor exposes:
//   - name(): a stable identifier for logging.
//   - sample_rate_hz(): the configured sample rate.
//   - healthy(): true if the last read succeeded within the expected interval.
//   - last_read_timestamp_ns(): nanoseconds since epoch of the last sample.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>
#include <string>

namespace thunders::sensors {

class Sensor {
public:
    Sensor() = default;
    virtual ~Sensor() = default;

    Sensor(const Sensor&)            = delete;
    Sensor& operator=(const Sensor&) = delete;

    /// Opens the device and configures the sample rate.
    /// Returns true on success, false on failure.
    virtual bool init() = 0;

    /// Pulls the latest sample (non-blocking). Returns true if a fresh
    /// sample was available.
    virtual bool read() = 0;

    /// Closes the device. Idempotent.
    virtual void close() = 0;

    /// Identity of this sensor for logging.
    virtual const std::string& name() const noexcept = 0;

    /// Configured sample rate, in Hz.
    virtual double sample_rate_hz() const noexcept = 0;

    /// True if the last read() call succeeded and the sensor has not
    /// missed more than the expected number of samples.
    virtual bool healthy() const noexcept = 0;

    /// Nanoseconds since epoch of the last successful read.
    virtual uint64_t last_read_timestamp_ns() const noexcept = 0;

    /// Returns a human-readable description of the sensor's current state.
    /// Used by the diagnostic dashboard.
    virtual std::string diagnostic_string() const {
        return name() + " @ " + std::to_string(sample_rate_hz()) + "Hz";
    }
};

} // namespace thunders::sensors
