/**
 * @file motor_driver.cpp
 * @brief Motor PWM driver implementation using Arduino-ESP32's `ledcWrite`.
 *
 * Uses four LEDC channels (0..3), two per motor. For a sign-preserving drive,
 * only one of the two channels per motor is ever non-zero at a time:
 *   forward  →  pwm_a = |pwm|, pwm_b = 0
 *   reverse  →  pwm_a = 0,        pwm_b = |pwm|
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "drivers/motor_driver.hpp"

#include <Arduino.h>

#include "board_config.hpp"
#include "diagnostics/logger.hpp"
#include "motor_config.hpp"
#include "pin_config.hpp"

namespace thunders::drivers {

namespace {
// LEDC channel assignments (4 channels total: 2 per motor).
constexpr std::uint8_t kLedcLeftA   = 0;
constexpr std::uint8_t kLedcLeftB   = 1;
constexpr std::uint8_t kLedcRightA  = 2;
constexpr std::uint8_t kLedcRightB  = 3;
constexpr const char* kTag = "[motor]";
}  // namespace

MotorDriver::MotorDriver()  = default;
MotorDriver::~MotorDriver() { disableAll(); }

bool MotorDriver::begin() {
    using namespace thunders::board;
    using namespace thunders::pins;
    using namespace thunders::config::motor;

    // Configure direction-enable pins as outputs.
    pinMode(kMotorLeftEnable,  OUTPUT);
    pinMode(kMotorRightEnable, OUTPUT);
    digitalWrite(kMotorLeftEnable,  HIGH);
    digitalWrite(kMotorRightEnable, HIGH);

    // Attach LEDC channels to the PWM pins.
    ledcAttachPin(kMotorLeftPwmA,   kLedcLeftA);
    ledcAttachPin(kMotorLeftPwmB,   kLedcLeftB);
    ledcAttachPin(kMotorRightPwmA,  kLedcRightA);
    ledcAttachPin(kMotorRightPwmB,  kLedcRightB);

    ledcSetup(kLedcLeftA,  kPwmFrequencyHz, kPwmResolutionBits);
    ledcSetup(kLedcLeftB,  kPwmFrequencyHz, kPwmResolutionBits);
    ledcSetup(kLedcRightA, kPwmFrequencyHz, kPwmResolutionBits);
    ledcSetup(kLedcRightB, kPwmFrequencyHz, kPwmResolutionBits);

    // Start with motors coasting (no current draw).
    setLeft(0);
    setRight(0);

    initialised_ = true;
    thunders::diagnostics::Logger::instance().info(
        "%s initialis (PWM %u Hz / %u-bit)", kTag, kPwmFrequencyHz, kPwmResolutionBits);
    return true;
}

MotorDriver::PwmPair MotorDriver::resolve(std::int16_t pwm, std::int8_t dir_sign) const {
    using namespace thunders::config::motor;
    PwmPair out{0, 0};
    if (pwm == 0) return out;
    // Apply mounting-direction sign.
    std::int32_t signed_pwm = static_cast<std::int32_t>(pwm) * dir_sign;
    // Clamp to ±kPwmMax.
    if (signed_pwm >  kPwmMax) signed_pwm =  kPwmMax;
    if (signed_pwm < -kPwmMax) signed_pwm = -kPwmMax;
    if (signed_pwm > 0) out.a = static_cast<std::uint16_t>( signed_pwm);
    else                out.b = static_cast<std::uint16_t>(-signed_pwm);
    return out;
}

void MotorDriver::setLeft(std::int16_t pwm) {
    if (!initialised_) return;
    const auto p = resolve(pwm, thunders::config::motor::kLeftDirSign);
    ledcWrite(kLedcLeftA, p.a);
    ledcWrite(kLedcLeftB, p.b);
}

void MotorDriver::setRight(std::int16_t pwm) {
    if (!initialised_) return;
    const auto p = resolve(pwm, thunders::config::motor::kRightDirSign);
    ledcWrite(kLedcRightA, p.a);
    ledcWrite(kLedcRightB, p.b);
}

void MotorDriver::coastLeft() {
    ledcWrite(kLedcLeftA,  0);
    ledcWrite(kLedcLeftB,  0);
}

void MotorDriver::coastRight() {
    ledcWrite(kLedcRightA, 0);
    ledcWrite(kLedcRightB, 0);
}

void MotorDriver::brakeLeft() {
    // Active brake = both channels HIGH (H-bridge short-circuit, fast decay).
    ledcWrite(kLedcLeftA,  thunders::config::motor::kPwmMax);
    ledcWrite(kLedcLeftB,  thunders::config::motor::kPwmMax);
}

void MotorDriver::brakeRight() {
    ledcWrite(kLedcRightA, thunders::config::motor::kPwmMax);
    ledcWrite(kLedcRightB, thunders::config::motor::kPwmMax);
}

void MotorDriver::disableAll() {
    if (!initialised_) return;
    coastLeft();
    coastRight();
    digitalWrite(thunders::pins::kMotorLeftEnable,  LOW);
    digitalWrite(thunders::pins::kMotorRightEnable, LOW);
}

}  // namespace thunders::drivers
