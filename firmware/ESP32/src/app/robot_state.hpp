/**
 * @file robot_state.hpp
 * @brief Thread-safe container for the global robot state snapshot.
 *
 * Every subsystem reads from (and a small set writes to) this struct. All
 * access goes through a `SemaphoreHandle_t` mutex so that high-priority
 * control loops and low-priority telemetry tasks can share data without
 * torn reads.
 *
 * Design rule: the contents are kept **small and POD-like** so a snapshot
 * can be copied cheaply under the mutex and used lock-free thereafter.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_ROBOT_STATE_HPP
#define THUNDERS_ROBOT_STATE_HPP

#include <cstdint>
#include <mutex>

#include "app_config.hpp"

namespace thunders::app {

struct RobotStateSnapshot {
    std::uint64_t timestampMs = 0;

    config::RobotMode mode          = config::RobotMode::kBoot;
    bool              safetyArmed    = true;
    bool              estopTriggered = false;
    bool              heartbeatAlive = false;
    float             batteryVoltageV = 0.0f;
    float             cpuLoadPercent  = 0.0f;
    float             freeHeapBytes   = 0.0f;

    // Body-frame motion state
    float linearVelocityMs  = 0.0f;
    float angularVelocityRs = 0.0f;
    float targetLinearMs    = 0.0f;
    float targetAngularRs  = 0.0f;

    // Per-wheel state
    float leftWheelRpm  = 0.0f;
    float rightWheelRpm = 0.0f;
    std::int32_t leftEncoderCount  = 0;
    std::int32_t rightEncoderCount = 0;

    // IMU
    float headingRad  = 0.0f;
    float pitchRad    = 0.0f;
    float rollRad     = 0.0f;
    bool  imuCalibrated = false;

    // Sensors
    float obstacleDistanceM = 0.0f;
    bool  obstacleDetected  = false;

    // Diagnostics counters
    std::uint32_t errors   = 0;
    std::uint32_t warnings = 0;
    std::uint32_t lastError = 0;
};

class RobotState {
public:
    RobotState();

    /// Take a consistent snapshot under the mutex. Cheap (memcpy).
    RobotStateSnapshot snapshot() const;

    /// Replace the whole state atomically. Most callers should use the
    /// targeted setters below instead.
    void setSnapshot(const RobotStateSnapshot& s);

    // ─── Targeted setters (lock + update + unlock) ────────────────────────
    void setMode(config::RobotMode m);
    void setEstop(bool triggered);
    void setHeartbeat(bool alive);
    void setBattery(float volts);
    void setVelocity(float lin_ms, float ang_rs);
    void setTargetVelocity(float lin_ms, float ang_rs);
    void setWheels(float left_rpm, float right_rpm,
                   std::int32_t left_count, std::int32_t right_count);
    void setImu(float heading, float pitch, float roll, bool calibrated);
    void setObstacle(float distance_m, bool detected);

private:
    mutable std::mutex mutex_;
    RobotStateSnapshot s_;
};

}  // namespace thunders::app

#endif  // THUNDERS_ROBOT_STATE_HPP
