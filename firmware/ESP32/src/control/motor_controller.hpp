/**
 * @file motor_controller.hpp
 * @brief Per-wheel closed-loop speed controller.
 *
 * Wraps two PidController instances (one per wheel) and translates wheel
 * RPM setpoints into PWM values via `MotorDriver::set{Left,Right}()`.
 * Reads wheel RPM from `EncoderDriver::readRpm()`.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_MOTOR_CONTROLLER_HPP
#define THUNDERS_MOTOR_CONTROLLER_HPP

#include <cstdint>

#include "control/pid_controller.hpp"
#include "drivers/encoder_driver.hpp"
#include "drivers/motor_driver.hpp"

namespace thunders::control {

class MotorController {
public:
    MotorController(drivers::MotorDriver& motor, drivers::EncoderDriver& enc);
    ~MotorController() = default;

    /// Configure PID gains & limits from motor_config.hpp.
    void begin();

    /// Set per-wheel RPM targets. Caller is responsible for rate-limiting.
    void setTargets(float left_rpm, float right_rpm);

    /// Read the current targets back.
    struct Targets { float left; float right; };
    Targets targets() const { return {targetLeft_, targetRight_}; }

    /// Run one control iteration. Should be called at the control-task
    /// period (typically 10 ms / 100 Hz).
    void update();

    /// Halt both motors immediately (used by e-stop).
    void halt();

private:
    drivers::MotorDriver&   motor_;
    drivers::EncoderDriver& enc_;
    PidController            pidLeft_;
    PidController            pidRight_;
    float                    targetLeft_  = 0.0f;
    float                    targetRight_ = 0.0f;
    std::uint32_t            lastUpdateMs_ = 0;
};

}  // namespace thunders::control

#endif  // THUNDERS_MOTOR_CONTROLLER_HPP
