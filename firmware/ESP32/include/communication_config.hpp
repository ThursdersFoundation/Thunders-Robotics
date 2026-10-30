/**
 * @file communication_config.hpp
 * @brief Configuration for every communication channel: Wi-Fi, MQTT, serial
 *        protocol, and ROS bridge.
 *
 * **Security note**: this file deliberately contains no credentials. Wi-Fi
 * SSID/password and MQTT broker credentials must come from a `secrets.local.hpp`
 * file (git-ignored) or from NVS at runtime — never hardcoded in the repo.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_COMMUNICATION_CONFIG_HPP
#define THUNDERS_COMMUNICATION_CONFIG_HPP

#include <cstdint>

namespace thunders::config::comms {

// ─── Wi-Fi (station mode) ─────────────────────────────────────────────────
namespace wifi {
    inline constexpr std::uint32_t kConnectTimeoutMs = 10'000;
    inline constexpr std::uint32_t kReconnectIntervalMs = 5'000;
    inline constexpr std::uint8_t  kMaxReconnectAttempts = 3;
    inline constexpr std::uint32_t kIpLeaseTimeMs      = 86'400'000;  // 24 h
}  // namespace wifi

// ─── MQTT (PubSubClient) ──────────────────────────────────────────────────
namespace mqtt {
    inline constexpr std::uint16_t kDefaultPort       = 1883;
    inline constexpr std::uint32_t kKeepAliveSec      = 30;
    inline constexpr std::uint32_t kSocketTimeoutMs   = 5'000;
    inline constexpr std::uint16_t kBufferSize        = 2048;
    inline constexpr std::uint16_t kMaxTopicLen       = 80;
    inline constexpr std::uint16_t kMaxPayloadLen     = 1024;
    inline constexpr std::uint8_t  kMaxRetries        = 5;
    inline constexpr std::uint32_t kRetryBackoffMs    = 2'000;

    // Topic namespace: thunders/{robot_id}/{category}
    inline constexpr const char* kTopicPrefix         = "thunders";
    inline constexpr const char* kTopicTelemetry      = "telemetry";
    inline constexpr const char* kTopicCommand         = "command";
    inline constexpr const char* kTopicStatus          = "status";
    inline constexpr const char* kTopicHealth           = "health";
    inline constexpr const char* kTopicHeartbeat        = "heartbeat";
}  // namespace mqtt

// ─── Serial binary protocol (USB-CDC / UART0 bridge) ─────────────────────
namespace serial {
    inline constexpr std::uint32_t kBaudRate         = 115200;
    inline constexpr std::uint16_t kMaxFrameBytes   = 256;
    inline constexpr std::uint16_t kRxBufBytes      = 1024;
    inline constexpr std::uint8_t  kFrameStartByte  = 0xAA;
    inline constexpr std::uint8_t  kFrameEndByte    = 0x55;
    inline constexpr std::uint8_t  kFrameEscapeByte = 0x1B;
    inline constexpr std::uint32_t kRxTimeoutMs     = 100;
}  // namespace serial

// ─── ROS bridge (rosbridge_suite WebSocket, optional) ───────────────────
namespace ros {
    inline constexpr std::uint16_t kDefaultPort      = 9090;
    inline constexpr std::uint32_t kHeartbeatPeriodMs = 1'000;
    inline constexpr std::uint32_t kCommandTimeoutMs  = 500;
    inline constexpr const char* kTopicCmdVel        = "/thunders/cmd_vel";
    inline constexpr const char* kTopicOdom          = "/thunders/odom";
    inline constexpr const char* kTopicImu           = "/thunders/imu";
    inline constexpr const char* kTopicScan          = "/thunders/scan";
}  // namespace ros

}  // namespace thunders::config::comms

#endif  // THUNDERS_COMMUNICATION_CONFIG_HPP
