/**
 * @file mqtt_client.hpp
 * @brief MQTT client wrapper (PubSubClient) for Thunders Robotic telemetry.
 *
 * Subscribes to the configured command topic and publishes periodic
 * telemetry JSON snapshots. Topic namespace follows:
 *     thunders/{robot_id}/command
 *     thunders/{robot_id}/telemetry
 *     thunders/{robot_id}/status
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_MQTT_CLIENT_HPP
#define THUNDERS_MQTT_CLIENT_HPP

#include <cstdint>

#include <WiFiClient.h>
#include <PubSubClient.h>

#include "communication/wifi_manager.hpp"

namespace thunders::communication {

class MqttClient {
public:
    MqttClient();
    ~MqttClient();

    /// Configure server, port, and topic strings. Does NOT connect.
    void begin();

    /// Periodic loop — must be called frequently (>= every 100 ms) by the
    /// MQTT task. Handles reconnection and dispatches incoming messages.
    void loop();

    /// Publish a JSON payload to the telemetry topic.
    void publishTelemetry(const char* json);

    /// Publish to a free-form topic under the robot's namespace.
    void publish(const char* subtopic, const char* payload,
                 bool retained = false);

    /// True if currently connected to the broker.
    bool isConnected() const { return pubsub_.connected(); }

private:
    void connect_();
    void onMessage_(char* topic, byte* payload, unsigned int length);

    WiFiClient     wifiClient_;
    PubSubClient  pubsub_;
    bool          initialised_ = false;
    std::uint32_t lastAttempt_ = 0;
    char          topicCmd_[80];
    char          topicTele_[80];
    char          topicStat_[80];
};

}  // namespace thunders::communication

#endif  // THUNDERS_MQTT_CLIENT_HPP
