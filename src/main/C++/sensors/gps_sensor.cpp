// =============================================================================
// thunders/sensors/gps_sensor.cpp
// -----------------------------------------------------------------------------
// Global Positioning System receiver.
//
// Outdoor robots use GPS for absolute position. This class wraps an NMEA
// 0183-compatible GPS receiver (e.g., u-blox NEO-M9N) and exposes:
//   - latitude_deg(), longitude_deg(): WGS-84 coordinates.
//   - altitude_m(): height above mean sea level.
//   - ground_speed_mps(): 2D speed over ground.
//   - heading_deg(): direction of travel (COG).
//   - hdop(): horizontal dilution of precision (lower is better).
//   - satellites_visible(): number of satellites used in the fix.
//   - fix_type(): NO_FIX, 2D, 3D, RTK_FLOAT, RTK_FIXED.
//
// The class parses NMEA sentences from a UART. Sentences are processed
// one at a time; partial sentences are buffered until a complete line is
// received.
//
// NOTE: GPS accuracy is typically 2-3 m without corrections, 0.5-1 m with
// SBAS, and <10 cm with RTK. For robots that need cm-level accuracy, use
// RTK and a base station; this class supports both modes.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>
#include <string>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class GPSSensor : public Sensor {
public:
    enum class FixType {
        NO_FIX   = 0,
        FIX_2D   = 1,
        FIX_3D   = 2,
        RTK_FLOAT= 3,
        RTK_FIXED= 4,
    };

    GPSSensor(std::string name,
              std::string uart_port,
              uint32_t    baud_rate,
              double      sample_rate_hz)
        : name_(std::move(name)),
          uart_port_(std::move(uart_port)),
          baud_rate_(baud_rate),
          sample_rate_hz_(sample_rate_hz) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- GPS-specific accessors -----------------------------------------
    double   latitude_deg()        const noexcept { return latitude_deg_; }
    double   longitude_deg()       const noexcept { return longitude_deg_; }
    double   altitude_m()          const noexcept { return altitude_m_; }
    double   ground_speed_mps()    const noexcept { return ground_speed_mps_; }
    double   heading_deg()         const noexcept { return heading_deg_; }
    double   hdop()                const noexcept { return hdop_; }
    uint32_t satellites_visible()  const noexcept { return satellites_; }
    FixType  fix_type()            const noexcept { return fix_type_; }

    /// True if the current fix is good enough for navigation (3D or better).
    bool has_valid_fix() const noexcept {
        return fix_type_ >= FixType::FIX_3D;
    }

    /// Returns the UTC time of the last fix, in milliseconds since epoch.
    uint64_t fix_time_utc_ms() const noexcept { return fix_time_utc_ms_; }

private:
    std::string  name_;
    std::string  uart_port_;
    uint32_t     baud_rate_;
    double       sample_rate_hz_;
    bool         healthy_       {false};
    uint64_t     last_ns_       {0};

    double       latitude_deg_      {0.0};
    double       longitude_deg_     {0.0};
    double       altitude_m_        {0.0};
    double       ground_speed_mps_  {0.0};
    double       heading_deg_       {0.0};
    double       hdop_              {99.0};
    uint32_t     satellites_        {0};
    FixType      fix_type_          {FixType::NO_FIX};
    uint64_t     fix_time_utc_ms_   {0};

    std::string  nmea_buffer_;  ///< accumulates partial NMEA sentences
};

} // namespace thunders::sensors
