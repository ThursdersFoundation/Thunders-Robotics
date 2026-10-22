// =============================================================================
// thunders/sensors/lidar_sensor.cpp
// -----------------------------------------------------------------------------
// 2D LiDAR scanner.
//
// Wraps a 2D LiDAR (e.g., RPLiDAR A1/A2, Slamtec S1, Hokuyo URG-04LX).
// Exposes a polar scan (vector of (range, angle) pairs) and a Cartesian
// point cloud derived from the scan. The scan is stored in robot frame,
// with angle 0 pointing forward (+X) and angles increasing CCW.
//
// Configuration:
//   - sample_rate_hz: scans per second (typically 5-15 Hz).
//   - max_range_m: maximum reliable range (typically 6-12 m).
//   - angle_min/max: the angular extent of the scan (typically [-pi, pi]).
//
// The read() call blocks until a new scan is available. For non-blocking
// use, poll last_read_timestamp_ns() and only call read() when it is older
// than (1 / sample_rate_hz) seconds.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class LiDARSensor : public Sensor {
public:
    struct PolarReading {
        double range_m;
        double angle_rad;
    };
    struct CartesianPoint {
        double x, y;
    };

    LiDARSensor(std::string name,
                std::string serial_port,
                double      sample_rate_hz,
                double      max_range_m,
                double      angle_min_rad = -M_PI,
                double      angle_max_rad =  M_PI)
        : name_(std::move(name)),
          serial_port_(std::move(serial_port)),
          sample_rate_hz_(sample_rate_hz),
          max_range_m_(max_range_m),
          angle_min_rad_(angle_min_rad),
          angle_max_rad_(angle_max_rad) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- LiDAR-specific accessors ---------------------------------------
    const std::vector<PolarReading>&     scan_polar()    const noexcept { return polar_; }
    const std::vector<CartesianPoint>&   scan_cartesian() const noexcept { return cartesian_; }

    double max_range_m()    const noexcept { return max_range_m_; }
    double angle_min_rad()  const noexcept { return angle_min_rad_; }
    double angle_max_rad()  const noexcept { return angle_max_rad_; }

    /// Returns the number of valid (non-inf, non-nan) readings in the
    /// latest scan.
    std::size_t valid_reading_count() const noexcept { return polar_.size(); }

    /// Returns the closest reading in the forward 90-degree cone, or
    /// max_range_m_ if no reading is present in that cone.
    double closest_forward_reading_m() const;

private:
    std::string                       name_;
    std::string                       serial_port_;
    double                            sample_rate_hz_;
    double                            max_range_m_;
    double                            angle_min_rad_;
    double                            angle_max_rad_;
    bool                              healthy_   {false};
    uint64_t                          last_ns_   {0};

    std::vector<PolarReading>         polar_;
    std::vector<CartesianPoint>       cartesian_;
};

} // namespace thunders::sensors
