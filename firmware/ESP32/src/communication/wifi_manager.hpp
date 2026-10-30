/**
 * @file wifi_manager.hpp
 * @brief Wi-Fi station-mode connection manager with auto-reconnect.
 *
 * Connects to the configured SSID in station mode, exposes the resulting
 * IP/MAC, and runs a reconnect loop. Credentials come from `secrets.local.hpp`
 * (git-ignored) — see `include/communication_config.hpp` for the protocol.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_WIFI_MANAGER_HPP
#define THUNDERS_WIFI_MANAGER_HPP

#include <cstdint>

namespace thunders::communication {

class WifiManager {
public:
    WifiManager() = default;
    ~WifiManager();

    /// Connect to the configured SSID. Returns false if no SSID configured.
    bool begin();

    /// Block until either connected or the connect-timeout expires.
    bool waitForConnection(std::uint32_t timeout_ms) const;

    /// Periodically invoked from the MQTT task — triggers reconnect if down.
    void loop();

    /// True iff the station is currently associated and has an IP.
    bool isConnected() const;

    /// Local IP as a string ("192.168.1.10") or "<disconnected>".
    const char* ipString() const;

    /// RSSI in dBm, or INT32_MIN if disconnected.
    std::int32_t rssi() const;

private:
    bool       initialised_ = false;
    std::uint32_t lastConnectAttempt_ = 0;
    char       ipBuf_[20] = {};
};

}  // namespace thunders::communication

#endif  // THUNDERS_WIFI_MANAGER_HPP
