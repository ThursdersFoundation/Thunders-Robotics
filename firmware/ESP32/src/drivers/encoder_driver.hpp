/**
 * @file encoder_driver.hpp
 * @brief Quadrature encoder reader using the ESP32 PCNT peripheral.
 *
 * Reads two-wheel odometry: instantaneous counts and wheel RPM. The PCNT
 * peripheral counts edges in hardware so the firmware does not need to
 * service every encoder interrupt in software — `readCounts()` simply reads
 * the hardware counters and clears them.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_ENCODER_DRIVER_HPP
#define THUNDERS_ENCODER_DRIVER_HPP

#include <cstdint>

namespace thunders::drivers {

class EncoderDriver {
public:
    EncoderDriver();
    ~EncoderDriver();

    bool begin();

    /// Read & reset the hardware counters. Returns delta counts since last call.
    struct Delta { std::int16_t left; std::int16_t right; };
    Delta readDelta();

    /// Cumulative signed counts since boot.
    struct Counts { std::int32_t left; std::int32_t right; };
    Counts readCounts() const;

    /// Instantaneous wheel speed in RPM. Requires the caller to invoke
    /// this at a known rate (typically the control task at 100 Hz).
    struct Rpm { float left; float right; };
    Rpm readRpm(std::uint32_t dt_ms);

private:
    bool initialised_ = false;
    std::int32_t leftTotal_  = 0;
    std::int32_t rightTotal_ = 0;
    std::int16_t lastLeft_   = 0;
    std::int16_t lastRight_  = 0;
};

}  // namespace thunders::drivers

#endif  // THUNDERS_ENCODER_DRIVER_HPP
