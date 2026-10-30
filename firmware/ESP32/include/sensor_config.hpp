/**
 * @file sensor_config.hpp
 * @brief Configuration for all onboard and external sensors.
 *
 * Covers the IMU (BNO055), ultrasonic ranger (HC-SR04), battery ADC, and the
 * quadrature encoders. Each section groups calibration, sample rate, and
 * filtering parameters so a single header edit can retune a sensor.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_SENSOR_CONFIG_HPP
#define THUNDERS_SENSOR_CONFIG_HPP

#include <cstdint>

namespace thunders::config::sensor {

// ─── IMU (BNO055, I²C @ 0x28 / 0x29) ────────────────────────────────────────
namespace imu {
    inline constexpr std::uint8_t  kI2cAddress       = 0x28;
    inline constexpr std::uint32_t kSamplePeriodMs   = 20;    // 50 Hz
    inline constexpr std::uint8_t  kOperationMode    = 0x0C;  // NDOF (fusion)
    inline constexpr float         kGyroRangeDps     = 2000.0f;
    inline constexpr float         kAccelRangeG      = 4.0f;
    inline constexpr float         kHeadingOffsetRad = 0.0f;  // calibrate per robot

    // EMA low-pass factor: 1.0 = no filter, ~0.2 = heavy smoothing
    inline constexpr float kFilterAlpha = 0.6f;
}  // namespace imu

// ─── Quadrature encoders (PCNT peripheral, ESP32-specific) ────────────────
namespace encoder {
    inline constexpr std::uint32_t kSamplePeriodMs  = 10;    // 100 Hz
    inline constexpr std::int16_t  kGlitchFilterNs  = 100;   // 100 ns
    inline constexpr std::uint16_t kPcntLimit       = 16384; // wrap-around
    inline constexpr float         kCountsToRpmFactor = 60000.0f /
        (kPcntLimit * 0.0f /* placeholder, see motor_config kCountsPerRevolution */);
    // NOTE: full conversion factor requires motor_config.hpp; computed at run
    // time in encoder_driver.cpp via kCountsPerRevolution (motor_config.hpp).
}  // namespace encoder

// ─── Ultrasonic ranger (HC-SR04) ───────────────────────────────────────────
namespace ultrasonic {
    inline constexpr std::uint32_t kTriggerPulseUs  = 10;    // 10 µs
    inline constexpr std::uint32_t kEchoTimeoutUs   = 30'000; // 30 ms (~5.1 m)
    inline constexpr std::uint32_t kSamplePeriodMs  = 100;   // 10 Hz
    inline constexpr float         kSpeedOfSoundMsUs = 0.0343f; // m/µs @ 20 °C
    inline constexpr float         kMinDistanceM    = 0.02f;  // 2 cm blind zone
    inline constexpr float         kMaxDistanceM    = 4.00f;  // 4 m realistic
    inline constexpr std::uint8_t  kMedianWindowSize = 5;
}  // namespace ultrasonic

// ─── Battery voltage monitor (ADC1_CH0 on GPIO 36) ────────────────────────
namespace battery {
    inline constexpr std::uint32_t kSamplePeriodMs  = 1000;  // 1 Hz
    inline constexpr float         kDividerRatio    = 2.0f;  // 100k/100k divider
    inline constexpr float         kFullVoltageV     = 8.4f;   // 2S Li-ion
    inline constexpr float         kEmptyVoltageV    = 6.4f;
    inline constexpr float         kLowVoltageV      = 7.0f;  // soft warning
    inline constexpr float         kCriticalVoltageV = 6.6f; // force shutdown
    inline constexpr std::uint8_t  kAdcSamplesAveraged = 8;
}  // namespace battery

}  // namespace thunders::config::sensor

#endif  // THUNDERS_SENSOR_CONFIG_HPP
