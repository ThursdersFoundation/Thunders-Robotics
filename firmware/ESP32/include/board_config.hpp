/**
 * @file board_config.hpp
 * @brief Board-level definitions: chip family, clock, flash size, etc.
 *
 * Each supported board (esp32dev, esp32s3, esp32c3) selects its own profile
 * via the THUNDERS_BOARD macro defined in platformio.ini. The rest of the
 * firmware should never call `#ifdef ESP32_S3` directly — it should use the
 * constexpr values defined here so the compiler can DCE unused branches.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_BOARD_CONFIG_HPP
#define THUNDERS_BOARD_CONFIG_HPP

#include <cstdint>

#if defined(THUNDERS_BOARD) && (THUNDERS_BOARD == "esp32dev")
    #define THUNDERS_BOARD_ESP32 1
#elif defined(THUNDERS_BOARD) && (THUNDERS_BOARD == "esp32s3")
    #define THUNDERS_BOARD_ESP32S3 1
#elif defined(THUNDERS_BOARD) && (THUNDERS_BOARD == "esp32c3")
    #define THUNDERS_BOARD_ESP32C3 1
#else
    #error "Unknown THUNDERS_BOARD — define it in platformio.ini build_flags"
#endif

namespace thunders::board {

#if defined(THUNDERS_BOARD_ESP32)
    inline constexpr const char* kBoardName       = "esp32dev";
    inline constexpr std::uint32_t kCpuFreqMhz    = 240;
    inline constexpr std::uint32_t kFlashSizeBytes = 4 * 1024 * 1024;  // 4 MB
    inline constexpr std::uint32_t kSramBytes      = 520 * 1024;       // 520 KB
    inline constexpr std::uint8_t  kCpuCores       = 2;
    inline constexpr bool          kHasBluetooth   = true;
    inline constexpr bool          kHasWifi        = true;
#elif defined(THUNDERS_BOARD_ESP32S3)
    inline constexpr const char* kBoardName       = "esp32s3";
    inline constexpr std::uint32_t kCpuFreqMhz    = 240;
    inline constexpr std::uint32_t kFlashSizeBytes = 8 * 1024 * 1024;  // 8 MB
    inline constexpr std::uint32_t kSramBytes      = 512 * 1024;
    inline constexpr std::uint8_t  kCpuCores       = 2;
    inline constexpr bool          kHasBluetooth   = false;
    inline constexpr bool          kHasWifi        = true;
#elif defined(THUNDERS_BOARD_ESP32C3)
    inline constexpr const char* kBoardName       = "esp32c3";
    inline constexpr std::uint32_t kCpuFreqMhz    = 160;
    inline constexpr std::uint32_t kFlashSizeBytes = 4 * 1024 * 1024;
    inline constexpr std::uint32_t kSramBytes      = 400 * 1024;
    inline constexpr std::uint8_t  kCpuCores       = 1;  // RISC-V single-core
    inline constexpr bool          kHasBluetooth   = true;
    inline constexpr bool          kHasWifi        = true;
#endif

// ─── I²C bus (shared by IMU and any external peripherals) ──────────────────
inline constexpr std::uint32_t kI2cFrequencyHz = 100'000;  // 100 kHz

// ─── PWM configuration (LEDC peripheral, used for motor drivers) ───────────
inline constexpr std::uint32_t kPwmFrequencyHz = 20'000;  // 20 kHz, ultrasonic
inline constexpr std::uint8_t  kPwmResolutionBits = 10;   // 0…1023
inline constexpr std::uint16_t kPwmMaxValue = (1u << kPwmResolutionBits) - 1u;

}  // namespace thunders::board

#endif  // THUNDERS_BOARD_CONFIG_HPP
