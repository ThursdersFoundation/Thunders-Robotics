/**
 * @file emergency_stop.hpp
 * @brief Hardware e-stop supervision and coordinated shutdown.
 *
 * Monitors the physical e-stop button (active-low input) plus several
 * software conditions (low battery, tip-over, lost heartbeat, watchdog
 * trigger) and forces the robot into `RobotMode::kEmergency` when any of
 * them trips. While in emergency mode the motors are disabled and a
 * manual reset is required before resuming operation.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_EMERGENCY_STOP_HPP
#define THUNDERS_EMERGENCY_STOP_HPP

#include <cstdint>

#include "control/motor_controller.hpp"
#include "control/velocity_controller.hpp"
#include "drivers/motor_driver.hpp"

namespace thunders::safety {

enum class EstopReason : std::uint8_t {
    kNone             = 0,
    kUserButton       = 1,   // hardware e-stop pressed
    kUserCommand      = 2,   // software command from MQTT / serial
    kLostHeartbeat    = 3,   // no telemetry from base in kHeartbeatGraceMs
    kLowBattery       = 4,   // battery below kCriticalBatteryVoltageV
    kTipOver          = 5,   // IMU pitch/roll exceeds threshold
    kObstacleStop      = 6,   // ultrasonic distance < kStopDistanceM
    kWatchdogTriggered = 7,
    kUserShutdown     = 8,   // graceful shutdown requested
};

class EmergencyStop {
public:
    EmergencyStop(drivers::MotorDriver& motor, control::VelocityController& vc)
        : motor_(motor), velCtrl_(vc) {}

    /// Configure the e-stop input pin, attach interrupt if needed.
    void begin();

    /// Periodically polled by the e-stop task. Checks all triggers.
    void poll();

    /// Programmatically trigger an e-stop. Safe to call from any task.
    void trigger(EstopReason reason);

    /// Reset the e-stop state back to armed — only valid if the underlying
    /// cause has been resolved AND `kRequireManualResetOnEstop` is false.
    void reset();

    bool         armed()     const { return armed_; }
    bool         triggered() const { return triggered_; }
    EstopReason  reason()    const { return reason_; }
    std::uint32_t triggerTimestamp() const { return triggerMs_; }

private:
    drivers::MotorDriver&             motor_;
    control::VelocityController&      velCtrl_;

    bool         armed_      = false;
    bool         triggered_  = false;
    EstopReason  reason_    = EstopReason::kNone;
    std::uint32_t triggerMs_ = 0;
    std::uint32_t lastButtonStateMs_ = 0;
    bool         lastButtonRaw_ = true;  // active-low, default HIGH
};

}  // namespace thunders::safety

#endif  // THUNDERS_EMERGENCY_STOP_HPP
