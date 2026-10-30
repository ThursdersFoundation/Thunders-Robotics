/**
 * @file wifi_manager.cpp
 * @brief Wi-Fi station-mode implementation built on WiFi.h (ESP-IDF).
 *
 * Credentials are injected via `THUNDERS_WIFI_SSID` / `THUNDERS_WIFI_PASS`
 * macros from a git-ignored `secrets.local.hpp` file at the top of the
 * include path. If the macros are not defined, the manager degrades to a
 * no-op and the rest of the firmware runs offline.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "communication/wifi_manager.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <snprintf.h>

#include "communication_config.hpp"
#include "diagnostics/logger.hpp"

namespace thunders::communication {

#if __has_include("secrets.local.hpp")
    #include "secrets.local.hpp"
    #ifndef THUNDERS_WIFI_SSID
        #define THUNDERS_WIFI_SSID ""
    #endif
    #ifndef THUNDERS_WIFI_PASS
        #define THUNDERS_WIFI_PASS ""
    #endif
#else
    #define THUNDERS_WIFI_SSID ""
    #define THUNDERS_WIFI_PASS ""
#endif

namespace {
constexpr const char* kTag = "[wifi]";
}

WifiManager::~WifiManager() {
    WiFi.disconnect(true, true);
}

bool WifiManager::begin() {
    using namespace thunders::config::comms::wifi;

    if (strlen(THUNDERS_WIFI_SSID) == 0) {
        thunders::diagnostics::Logger::instance().warn(
            "%s no SSID configured — running offline", kTag);
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // lower latency at slight power cost
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);

    thunders::diagnostics::Logger::instance().info(
        "%s connecting to SSID '%s'…", kTag, THUNDERS_WIFI_SSID);
    WiFi.begin(THUNDERS_WIFI_SSID, THUNDERS_WIFI_PASS);
    initialised_ = true;
    lastConnectAttempt_ = millis();
    return true;
}

bool WifiManager::waitForConnection(std::uint32_t timeout_ms) const {
    if (!initialised_) return false;
    const std::uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
        delay(100);
    }
    return WiFi.status() == WL_CONNECTED;
}

void WifiManager::loop() {
    using namespace thunders::config::comms::wifi;
    if (!initialised_) return;

    if (WiFi.status() != WL_CONNECTED) {
        const std::uint32_t now = millis();
        if (now - lastConnectAttempt_ >= kReconnectIntervalMs) {
            thunders::diagnostics::Logger::instance().warn(
                "%s reconnect attempt (status=%d)", kTag, WiFi.status());
            WiFi.reconnect();
            lastConnectAttempt_ = now;
        }
    }
}

bool WifiManager::isConnected() const {
    return initialised_ && WiFi.status() == WL_CONNECTED;
}

const char* WifiManager::ipString() const {
    if (!isConnected()) return "<disconnected>";
    const IPAddress ip = WiFi.localIP();
    snprintf(ipBuf_, sizeof(ipBuf_), "%u.%u.%u.%u",
             ip[0], ip[1], ip[2], ip[3]);
    return ipBuf_;
}

std::int32_t WifiManager::rssi() const {
    if (!isConnected()) return INT32_MIN;
    return WiFi.RSSI();
}

}  // namespace thunders::communication
