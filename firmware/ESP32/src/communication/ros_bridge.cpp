/**
 * @file ros_bridge.cpp
 * @brief rosbridge_suite WebSocket bridge skeleton.
 *
 * This is a deliberately minimal skeleton: it parses the broker URL into a
 * host:port pair and exposes `publish*()` entry points that callers would
 * fill in with actual WebSocket + JSON-RPC handling using, e.g., the
 * ArduinoWebsockets library or a hand-rolled client over WiFiClientSecure.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "communication/ros_bridge.hpp"

#include <Arduino.h>
#include <snprintf.h>
#include <string.h>

#include "app_config.hpp"
#include "communication_config.hpp"
#include "diagnostics/logger.hpp"

namespace thunders::communication {

namespace {
constexpr const char* kTag = "[ros_bridge]";

bool parseUrl(const char* url, char* host_out, std::size_t host_cap,
              std::uint16_t& port_out) {
    // Accept "ws://host[:port]" only.
    if (strncmp(url, "ws://", 5) != 0) return false;
    const char* p = url + 5;
    const char* colon = strchr(p, ':');
    const char* end   = p + strlen(p);
    if (colon) {
        const std::size_t hlen = static_cast<std::size_t>(colon - p);
        if (hlen >= host_cap) return false;
        memcpy(host_out, p, hlen);
        host_out[hlen] = '\0';
        port_out = static_cast<std::uint16_t>(atoi(colon + 1));
    } else {
        const std::size_t hlen = static_cast<std::size_t>(end - p);
        if (hlen >= host_cap) return false;
        memcpy(host_out, p, hlen);
        host_out[hlen] = '\0';
        port_out = thunders::config::comms::ros::kDefaultPort;
    }
    return true;
}
}  // namespace

RosBridge::RosBridge()  = default;
RosBridge::~RosBridge() = default;

bool RosBridge::begin(const char* url) {
    if (!thunders::config::kEnableRosBridge) {
        thunders::diagnostics::Logger::instance().info(
            "%s disabled at compile-time (kEnableRosBridge=false)", kTag);
        return false;
    }
    if (!parseUrl(url, host_, sizeof(host_), port_)) {
        thunders::diagnostics::Logger::instance().error(
            "%s invalid broker URL: %s", kTag, url);
        return false;
    }
    thunders::diagnostics::Logger::instance().info(
        "%s broker %s:%u ready (init connection in loop())", kTag, host_, port_);
    initialised_ = true;
    return true;
}

void RosBridge::loop() {
    if (!initialised_) return;
    // Skeleton: maintain WebSocket keep-alive here. In production, call
    // wsClient.poll() and dispatch incoming JSON-RPC frames to handlers.
}

void RosBridge::publishOdometry(const char* /*json*/) {
    if (!connected_) return;
    // Skeleton: send JSON over WebSocket with op="publish" + topic.
}

void RosBridge::publishImu(const char* /*json*/) {
    if (!connected_) return;
}

void RosBridge::publishScan(float distance_m) {
    if (!connected_) return;
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"op\":\"publish\",\"topic\":\"%s\",\"msg\":{\"range\":%.3f}}",
             thunders::config::comms::ros::kTopicScan, distance_m);
    (void)buf;  // wsClient.send(buf) in production.
}

}  // namespace thunders::communication
