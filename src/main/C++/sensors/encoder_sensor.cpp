// =============================================================================
// thunders/sensors/encoder_sensor.cpp
// -----------------------------------------------------------------------------
// Quadrature encoder reader.
//
// Quadrature encoders have two channels (A and B) that are 90 degrees out
// of phase. By counting edges and tracking the phase relationship, the
// encoder reports both position (in counts) and direction (CW or CCW).
//
// This class wraps the hardware encoder counter and exposes:
//   - position_counts(): cumulative count since reset.
//   - velocity_counts_per_s(): rate computed over a fixed window.
//   - position_radians(): position converted using counts_per_rev and
//     gear_ratio.
//   - velocity_rad_per_s(): rate converted to rad/s.
//
// The velocity is computed using a moving average over the last N samples
// to suppress quantisation noise at low speeds.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <deque>
#include <cstdint>
#include <string>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class EncoderSensor : public Sensor {
public:
    EncoderSensor(std::string name,
                  uint8_t     channel_a,
                  uint8_t     channel_b,
                  double      counts_per_rev,
                  double      gear_ratio,
                  bool        inverted,
                  double      sample_rate_hz)
        : name_(std::move(name)),
          channel_a_(channel_a),
          channel_b_(channel_b),
          counts_per_rev_(counts_per_rev),
          gear_ratio_(gear_ratio),
          inverted_(inverted),
          sample_rate_hz_(sample_rate_hz) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- encoder-specific accessors -------------------------------------
    int32_t  position_counts()       const noexcept { return position_counts_; }
    double   velocity_counts_per_s() const noexcept;
    double   position_radians()      const noexcept;
    double   velocity_rad_per_s()    const noexcept;

    /// Resets the cumulative position to zero.
    void reset() { position_counts_ = 0; }

    /// Sets a software offset such that position_radians() returns the
    /// given value at the current physical position.
    void set_position_radians(double rads);

    // --- configuration ---------------------------------------------------
    void set_inverted(bool inv) { inverted_ = inv; }
    void set_counts_per_rev(double c) { counts_per_rev_ = c; }
    void set_gear_ratio(double g) { gear_ratio_ = g; }

private:
    std::string         name_;
    uint8_t             channel_a_;
    uint8_t             channel_b_;
    double              counts_per_rev_;
    double              gear_ratio_;
    bool                inverted_;
    double              sample_rate_hz_;
    bool                healthy_       {false};
    uint64_t            last_ns_       {0};

    int32_t             position_counts_    {0};
    int32_t             last_position_      {0};
    double              offset_radians_     {0.0};
    std::deque<double>  velocity_window_;    ///< recent velocities for moving avg
    static constexpr std::size_t WINDOW_SIZE = 8;
};

} // namespace thunders::sensors
