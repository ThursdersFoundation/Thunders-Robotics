// =============================================================================
// thunders/sensors/infrared_sensor.cpp
// -----------------------------------------------------------------------------
// Single-point infrared distance sensor (e.g., Sharp GP2Y0A21YK).
//
// IR sensors output an analog voltage that is inversely proportional to
// distance: closer objects produce higher voltage. The exact relationship
// is non-linear and varies by sensor model; this class applies a
// model-specific calibration curve to convert the raw voltage to metres.
//
// IR sensors are useful for short-range (10-80 cm) detection where
// ultrasonic sensors struggle (e.g., detecting thin objects that do not
// reflect sound well). They are blinded by direct sunlight and by black
// surfaces that absorb IR.
//
// The reading is filtered through an exponential moving average (EMA) to
// suppress noise; the filter coefficient is configurable.
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

class InfraredSensor : public Sensor {
public:
    InfraredSensor(std::string name,
                   uint8_t     analog_port,
                   double      sample_rate_hz,
                   double      min_range_m,
                   double      max_range_m)
        : name_(std::move(name)),
          analog_port_(analog_port),
          sample_rate_hz_(sample_rate_hz),
          min_range_m_(min_range_m),
          max_range_m_(max_range_m) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- IR-specific accessors ------------------------------------------
    double distance_m() const noexcept { return filtered_distance_m_; }
    double raw_voltage() const noexcept { return raw_voltage_; }
    double min_range_m() const noexcept { return min_range_m_; }
    double max_range_m() const noexcept { return max_range_m_; }

    /// Sets the EMA filter coefficient in [0, 1]. 0 = no filtering,
    /// 1 = ignore new readings entirely. Default 0.3.
    void set_filter_alpha(double a) { alpha_ = std::clamp(a, 0.0, 1.0); }

    /// True if the latest reading indicates an object within min_range_m.
    /// Used by the intake subsystem to detect a game piece.
    bool object_detected(double threshold_m) const noexcept {
        return filtered_distance_m_ < threshold_m;
    }

private:
    /// Converts the raw analog voltage to metres using the standard
    /// Sharp GP2Y0A21YK model: d = 27 / (v - 0.5). Subclasses may override.
    virtual double voltage_to_metres(double v) const {
        if (v <= 0.5) return max_range_m_;
        return 27.0 / (v - 0.5) / 100.0;  // cm -> m
    }

    std::string  name_;
    uint8_t      analog_port_;
    double       sample_rate_hz_;
    double       min_range_m_;
    double       max_range_m_;
    bool         healthy_   {false};
    uint64_t     last_ns_   {0};

    double       raw_voltage_           {0.0};
    double       raw_distance_m_        {0.0};
    double       filtered_distance_m_   {0.0};
    double       alpha_                 {0.3};
};

} // namespace thunders::sensors
