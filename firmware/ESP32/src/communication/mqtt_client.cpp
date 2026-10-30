/**
 * @file mqtt_client.cpp
 * @brief MQTT client implementation built on PubSubClient.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "communication/mqtt_client.hpp"

#include <Arduino.h>
#include <snprintf.h>
#include <string.h>

#include "app_config.hpp"
#include "communication_config.hpp"
#include "diagnostics/logger.hpp"

namespace thunders::communication {

#if __has_include("secrets.local.hpp")
    #include "secrets.local.hpp"
    #ifndef THUNDERS_MQTT_HOST
        #define THUNDERS_MQTT_HOST ""
    #endif
    #ifndef THUNDERS_MQTT_PORT
        #define THUNDERS_MQTT_PORT 1883
    #endif
    #ifndef THUNDERS_MQTT_USER
        #define THUNDERS_MQTT_USER ""
    #endif
    #ifndef THUNDERS_MQTT_PASS
        #define THUNDERS_MQTT_PASS ""
    #endif
#else
    #define THUNDERS_MQTT_HOST ""
    #define THUNDERS_MQTT_PORT 1883
    #define THUNDERS_MQTT_USER ""
    #define THUNDERS_MQTT_PASS ""
#endif

namespace {
constexpr const char* kTag = "[mqtt]";
}  // namespace

MqttClient::MqttClient() {
    pubsub_.setClient(wifiClient_);
    pubsub_.setBufferSize(thunders::config::comms::mqtt::kBufferSize);
    pubsub_.setKeepAlive(thunders::config::comms::mqtt::kKeepAliveSec);
    pubsub_.setSocketTimeout(thunders::config::comms::mqtt::kSocketTimeoutMs / 1000);
}

MqttClient::~MqttClient() {
    pubsub_.disconnect();
}

void MqttClient::begin() {
    using namespace thunders::config::comms::mqtt;
    using namespace thunders::config;

    snprintf(topicCmd_,  sizeof(topicCmd_),  "%s/%s/%s",
             kTopicPrefix, kRobotName, kTopicCommand);
    snprintf(topicTele_, sizeof(topicTele_), "%s/%s/%s",
             kTopicPrefix, kRobotName, kTopicTelemetry);
    snprintf(topicStat_, sizeof(topicStat_), "%s/%s/%s",
             kTopicPrefix, kRobotName, kTopicStatus);

    if (strlen(THUNDERS_MQTT_HOST) == 0) {
        thunders::diagnostics::Logger::instance().warn(
            "%s no broker configured — disabled", kTag);
        return;
    }

    pubsub_.setServer(THUNDERS_MQTT_HOST, THUNDERS_MQTT_PORT);
    pubsub_.setCallback([this](char* t, byte* p, unsigned int l) {
        this->onMessage_(t, p, l);
    });
    initialised_ = true;
    thunders::diagnostics::Logger::instance().info(
        "%s broker=%s:%u cmd=%s", kTag, THUNDERS_MQTT_HOST,
        THUNDERS_MQTT_PORT, topicCmd_);
}

void MqttClient::connect_() {
    using namespace thunders::config::comms::mqtt;
    using namespace thunders::config;

    thunders::diagnostics::Logger::instance().info(
        "%s connecting to broker as '%s'…", kTag, kRobotName);

    bool ok;
    if (strlen(THUNDERS_MQTT_USER) > 0) {
        ok = pubsub_.connect(kRobotName,
                              THUNDERS_MQTT_USER, THUNDERS_MQTT_PASS,
                              topicStat_, /*qos=*/1, /*retained=*/true,
                              /*will=*/"offline");
    } else {
        ok = pubsub_.connect(kRobotName,
                              topicStat_, /*qos=*/1, /*retained=*/true,
                              "offline");
    }
    if (ok) {
        thunders::diagnostics::Logger::instance().info(
            "%s connected — subscribing to %s", kTag, topicCmd_);
        pubsub_.subscribe(topicCmd_);
        publish(topicStat_, "online", /*retained=*/true);
    } else {
        thunders::diagnostics::Logger::instance().warn(
            "%s connect failed (state=%d)", kTag, pubsub_.state());
    }
}

void MqttClient::loop() {
    using namespace thunders::config::comms::mqtt;

    if (!initialised_) return;
    if (!pubsub_.connected()) {
        const std::uint32_t now = millis();
        if (now - lastAttempt_ < kRetryBackoffMs) return;
        lastAttempt_ = now;
        connect_();
        return;
    }
    pubsub_.loop();
}

void MqttClient::publishTelemetry(const char* json) {
    if (!pubsub_.connected()) return;
    pubsub_.publish(topicTele_, json);
}

void MqttClient::publish(const char* subtopic, const char* payload, bool retained) {
    if (!pubsub_.connected()) return;
    using namespace thunders::config::comms::mqtt;
    using namespace thunders::config;
    char topic[80];
    snprintf(topic, sizeof(topic), "%s/%s/%s",
             kTopicPrefix, kRobotName, subtopic);
    pubsub_.publish(topic, payload, retained);
}

void MqttClient::onMessage_(char* /*topic*/, byte* payload, unsigned int length) {
    // Production: parse JSON with ArduinoJson and dispatch the command
    // (CmdVel, SetMode, Estop) to the RobotApp.
    char buf[256];
    const unsigned int n = (length < sizeof(buf) - 1) ? length
                                                      : sizeof(buf) - 1;
    memcpy(buf, payload, n);
    buf[n] = '\0';
    thunders::diagnostics::Logger::instance().info(
        "%s RX (%u B): %s", kTag, n, buf);
}

}  // namespace thunders::communication
