/**
 * @file robot_state.cpp
 * @brief Thread-safe robot state implementation.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "app/robot_state.hpp"

#include <Arduino.h>

namespace thunders::app {

RobotState::RobotState() = default;

RobotStateSnapshot RobotState::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return s_;
}

void RobotState::setSnapshot(const RobotStateSnapshot& s) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_ = s;
}

void RobotState::setMode(config::RobotMode m) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.mode = m;
    s_.timestampMs = millis();
}

void RobotState::setEstop(bool triggered) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.estopTriggered = triggered;
    if (triggered) s_.mode = config::RobotMode::kEmergency;
    s_.timestampMs = millis();
}

void RobotState::setHeartbeat(bool alive) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.heartbeatAlive = alive;
    s_.timestampMs = millis();
}

void RobotState::setBattery(float volts) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.batteryVoltageV = volts;
    s_.timestampMs = millis();
}

void RobotState::setVelocity(float lin_ms, float ang_rs) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.linearVelocityMs  = lin_ms;
    s_.angularVelocityRs = ang_rs;
    s_.timestampMs = millis();
}

void RobotState::setTargetVelocity(float lin_ms, float ang_rs) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.targetLinearMs = lin_ms;
    s_.targetAngularRs = ang_rs;
    s_.timestampMs = millis();
}

void RobotState::setWheels(float left_rpm, float right_rpm,
                            std::int32_t left_count, std::int32_t right_count) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.leftWheelRpm  = left_rpm;
    s_.rightWheelRpm = right_rpm;
    s_.leftEncoderCount  = left_count;
    s_.rightEncoderCount = right_count;
    s_.timestampMs = millis();
}

void RobotState::setImu(float heading, float pitch, float roll, bool calibrated) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.headingRad   = heading;
    s_.pitchRad     = pitch;
    s_.rollRad      = roll;
    s_.imuCalibrated = calibrated;
    s_.timestampMs = millis();
}

void RobotState::setObstacle(float distance_m, bool detected) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.obstacleDistanceM = distance_m;
    s_.obstacleDetected   = detected;
    s_.timestampMs = millis();
}

}  // namespace thunders::app
