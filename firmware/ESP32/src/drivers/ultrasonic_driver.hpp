/**
 * @file ultrasonic_driver.hpp
 * @brief HC-SR04 ultrasonic distance driver with median filtering.
 *
 * Reads the distance to the nearest obstacle in front of the robot. The
 * HC-SR04 returns a 5 V echo pulse which must be divided down (or level-
 * shifted) before being applied to a 3.3 V ESP32 input — see the wiring
 * notes in esp32/README.md.
 *
 * The driver applies a small median filter over the last N samples to
 * suppress the occasional spurious reading caused by acoustic echoes.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_ULTRASONIC_DRIVER_HPP
#define THUNDERS_ULTRASONIC_DRIVER_HPP

#include <cstdint>

namespace thunders::drivers {

class UltrasonicDriver {
public:
    UltrasonicDriver();
    ~UltrasonicDriver();

    bool begin();

    /// Read a filtered distance in meters. Returns kMaxDistanceM if no
    /// obstacle is detected within the configured range.
    float sample();

    /// True if the last successful reading indicated an obstacle within
    /// `kDangerDistanceM` (safety_config.hpp).
    bool obstacleNear() const;

private:
    bool  initialised_  = false;
    float lastReading_ = 0.0f;
};

}  // namespace thunders::drivers

#endif  // THUNDERS_ULTRASONIC_DRIVER_HPP
