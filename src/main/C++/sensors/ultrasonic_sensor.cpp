// =============================================================================
// thunders/sensors/ultrasonic_sensor.cpp
// -----------------------------------------------------------------------------
// Single-point ultrasonic distance sensor (e.g., HC-SR04, MaxBotix LV).
//
// Ultrasonic sensors measure the time-of-flight of an ultrasonic pulse and
// convert it to distance using the speed of sound in air (343 m/s at 20 C).
// They are cheap and robust but have poor angular resolution (~15 degrees
// beam width) and false readings when the target is highly absorbent
// (e.g., foam) or highly reflective at an oblique angle.
//
// This class wraps either an analog ultrasonic (MaxBotix, which reports
// voltage proportional to range) or a digital ping/echo ultrasonic
// (HC-SR04, which requires a trigger pulse and measures echo width). The
// two modes are selected at construction time.
//
// The reading is filtered through a simple median-of-3 filter to suppress
// the occasional outlier produced by multi-path reflections.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <string>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class UltrasonicSensor : public Sensor {
public:
    enum class Type { ANALOG, DIGITAL };

    UltrasonicSensor(std::string name,
                     Type        type,
                     uint8_t     analog_port,
                     uint8_t     trigger_pin,
                     uint8_t     echo_pin,
                     double      sample_rate_hz,
                     double      max_range_m)
        : name_(std::move(name)),
          type_(type),
          analog_port_(analog_port),
          trigger_pin_(trigger_pin),
          echo_pin_(echo_pin),
          sample_rate_hz_(sample_rate_hz),
          max_range_m_(max_range_m) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- ultrasonic-specific accessors ----------------------------------
    double distance_m() const noexcept {
        // Median of the last 3 readings.
        if (history_.empty()) return max_range_m_;
        std::deque<double> copy = history_;
        std::sort(copy.begin(), copy.end());
        return copy[copy.size() / 2];
    }

    double raw_distance_m() const noexcept { return raw_distance_m_; }
    double max_range_m()    const noexcept { return max_range_m_; }
    Type   type()           const noexcept { return type_; }

    /// Sets the speed of sound used for the time-of-flight calculation.
    /// Defaults to 343 m/s (air at 20 C). Override for non-standard
    /// environments (e.g., helium mixtures).
    void set_speed_of_sound_mps(double v) { speed_of_sound_mps_ = v; }

private:
    std::string         name_;
    Type                type_;
    uint8_t             analog_port_;
    uint8_t             trigger_pin_;
    uint8_t             echo_pin_;
    double              sample_rate_hz_;
    double              max_range_m_;
    bool                healthy_   {false};
    uint64_t            last_ns_   {0};

    double              raw_distance_m_     {0.0};
    double              speed_of_sound_mps_ {343.0};
    std::deque<double>  history_;  ///< rolling window for median filter
};

} // namespace thunders::sensors
