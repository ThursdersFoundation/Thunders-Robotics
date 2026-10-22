// =============================================================================
// thunders/sensors/force_sensor.cpp
// -----------------------------------------------------------------------------
// Single-axis force / load cell sensor.
//
// Used for grippers (to detect when a game piece is grabbed without
// crushing it), for legged robots (to detect ground contact), and for
// elevator overload protection. Wraps a strain-gauge load cell amplified
// by an HX711 (24-bit ADC) or similar.
//
// Calibration procedure:
//   1. With no load applied, call zero() to record the tare.
//   2. Apply a known reference mass (e.g., 1 kg).
//   3. Call calibrate(known_newtons) to compute the scale factor.
//   4. Repeat steps 2-3 with several masses for best accuracy.
//
// The reading is filtered through an EMA to suppress ADC noise; the
// filter coefficient is configurable. A threshold detector is included
// for convenience: above_threshold() returns true when the latest
// reading exceeds the configured threshold.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class ForceSensor : public Sensor {
public:
    ForceSensor(std::string name,
                uint8_t     data_pin,
                uint8_t     clock_pin,
                double      sample_rate_hz,
                double      max_force_n)
        : name_(std::move(name)),
          data_pin_(data_pin),
          clock_pin_(clock_pin),
          sample_rate_hz_(sample_rate_hz),
          max_force_n_(max_force_n) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- force-specific accessors ---------------------------------------
    double force_n()          const noexcept { return filtered_force_n_; }
    double raw_force_n()      const noexcept { return raw_force_n_; }
    double max_force_n()      const noexcept { return max_force_n_; }

    /// True if the latest reading exceeds the configured threshold.
    bool above_threshold() const noexcept {
        return std::abs(filtered_force_n_) >= threshold_n_;
    }

    // --- calibration -----------------------------------------------------
    /// Records the current reading as the zero-force reference.
    void zero() { tare_counts_ = last_raw_counts_; }

    /// Sets the scale factor (newtons per count) using the most recent
    /// reading and the known applied force.
    void calibrate(double known_newtons) {
        const int32_t diff = last_raw_counts_ - tare_counts_;
        if (diff != 0) {
            scale_n_per_count_ = known_newtons / std::abs(diff);
        }
    }

    // --- configuration ---------------------------------------------------
    void set_threshold_n(double n) { threshold_n_ = n; }
    void set_filter_alpha(double a) { alpha_ = std::clamp(a, 0.0, 1.0); }
    void set_scale_n_per_count(double s) { scale_n_per_count_ = s; }

    double threshold_n()       const noexcept { return threshold_n_; }
    double scale_n_per_count() const noexcept { return scale_n_per_count_; }

private:
    std::string  name_;
    uint8_t      data_pin_;
    uint8_t      clock_pin_;
    double       sample_rate_hz_;
    double       max_force_n_;
    bool         healthy_   {false};
    uint64_t     last_ns_   {0};

    int32_t      last_raw_counts_    {0};
    int32_t      tare_counts_        {0};
    double       scale_n_per_count_  {1.0};
    double       raw_force_n_        {0.0};
    double       filtered_force_n_   {0.0};
    double       alpha_              {0.3};
    double       threshold_n_        {5.0};
};

} // namespace thunders::sensors
