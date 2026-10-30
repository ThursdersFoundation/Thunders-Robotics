/**
 * @file pin_config.hpp
 * @brief Centralised GPIO pin assignments for the Thunders Robotic ESP32 board.
 *
 * **Do not** sprinkle magic pin numbers across driver files. Always route
 * through these `constexpr` constants so a board respin only requires editing
 * this single header.
 *
 * Default profile matches the ESP32-WROOM reference board documented in
 * esp32/README.md. Alternate boards (esp32s3, esp32c3) override via the
 * preprocessor switch at the top.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_PIN_CONFIG_HPP
#define THUNDERS_PIN_CONFIG_HPP

#include <cstdint>

#include "board_config.hpp"

namespace thunders::pins {

// ─── Left motor driver (e.g. TB6612FNG / DRV8833 channel A) ────────────────
inline constexpr std::uint8_t kMotorLeftPwmA  = 25;  // LEDC channel 0
inline constexpr std::uint8_t kMotorLeftPwmB  = 26;  // LEDC channel 1
inline constexpr std::uint8_t kMotorLeftEnable = 27;

// ─── Right motor driver ────────────────────────────────────────────────────
inline constexpr std::uint8_t kMotorRightPwmA  = 14;  // LEDC channel 2
inline constexpr std::uint8_t kMotorRightPwmB  = 15;  // LEDC channel 3
inline constexpr std::uint8_t kMotorRightEnable = 12;

// ─── Quadrature encoders (interrupt-capable, PCNT-friendly) ───────────────
inline constexpr std::uint8_t kEncoderLeftA  = 34;
inline constexpr std::uint8_t kEncoderLeftB  = 35;
inline constexpr std::uint8_t kEncoderRightA = 32;
inline constexpr std::uint8_t kEncoderRightB = 33;

// ─── I²C bus (IMU: BNO055, optional OLED display) ──────────────────────────
inline constexpr std::uint8_t kI2cScl = 22;
inline constexpr std::uint8_t kI2cSda = 21;

// ─── Ultrasonic ranger (HC-SR04) ───────────────────────────────────────────
inline constexpr std::uint8_t kUltrasonicTrigger = 4;
inline constexpr std::uint8_t kUltrasonicEcho    = 16;

// ─── Status LED (on-board) ────────────────────────────────────────────────
inline constexpr std::uint8_t kStatusLed = 2;

// ─── Emergency-stop input (active-low, hardware pull-up required) ─────────
inline constexpr std::uint8_t kEstopInput = 17;

// ─── Optional auxiliary inputs ─────────────────────────────────────────────
inline constexpr std::uint8_t kUserButton   = 0;   // BOOT button
inline constexpr std::uint8_t kBuzzerPin    = 13;
inline constexpr std::uint8_t kBatterySense = 36;  // ADC1_CH0, 12-bit

// ─── SPI bus (optional: SD card, external flash) ─────────────────────────
inline constexpr std::uint8_t kSpiSclk = 18;
inline constexpr std::uint8_t kSpiMiso = 19;
inline constexpr std::uint8_t kSpiMosi = 23;
inline constexpr std::uint8_t kSpiCs   = 5;

// ─── ESP32-S3 / ESP32-C3 overrides ─────────────────────────────────────────
#if defined(THUNDERS_BOARD_ESP32S3)
    #undef  kStatusLed
    inline constexpr std::uint8_t kStatusLed = 38;  // on-board RGB on S3 devkit
#elif defined(THUNDERS_BOARD_ESP32C3)
    #undef  kStatusLed
    inline constexpr std::uint8_t kStatusLed = 8;   // on-board LED on C3 devkit
    #undef  kEncoderLeftA
    #undef  kEncoderLeftB
    inline constexpr std::uint8_t kEncoderLeftA  = 1;   // C3 GPIO 34/35 don't exist
    inline constexpr std::uint8_t kEncoderLeftB  = 2;
#endif

}  // namespace thunders::pins

#endif  // THUNDERS_PIN_CONFIG_HPP
