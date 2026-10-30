/**
 * @file motor_driver.hpp
 * @brief Hardware abstraction over the motor PWM driver (TB6612FNG / DRV8833).
 *
 * Exposes simple `setLeft(int16 pwm)` / `setRight(int16 pwm)` calls with sign-
 * preserving PWM. Direction inversion is handled at this layer using the
 * `kLeftDirSign` / `kRightDirSign` constants from motor_config.hpp so the rest
 * of the firmware can assume positive PWM = forward motion.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_MOTOR_DRIVER_HPP
#define THUNDERS_MOTOR_DRIVER_HPP

#include <cstdint>

namespace thunders::drivers {

class MotorDriver {
public:
    MotorDriver();
    ~MotorDriver();

    /// Configure PWM channels, GPIO modes. Returns false on hard failure.
    bool begin();

    /// Apply sign-preserving PWM (-1023..+1023) to the left motor.
    /// Negative = reverse. Outside range is clamped.
    void setLeft(std::int16_t pwm);

    /// Apply sign-preserving PWM (-1023..+1023) to the right motor.
    void setRight(std::int16_t pwm);

    /// Coast (Hi-Z both outputs) — lets the wheels spin freely.
    void coastLeft();
    void coastRight();

    /// Active brake — short motor winding for fast deceleration.
    void brakeLeft();
    void brakeRight();

    /// Disable both motors entirely (used during e-stop).
    void disableAll();

private:
    /// Map a signed PWM value to the two-channel PWM driver.
    /// Returns (pwm_a, pwm_b) where one channel is always 0.
    struct PwmPair { std::uint16_t a; std::uint16_t b; };
    PwmPair resolve(std::int16_t pwm, std::int8_t dir_sign) const;

    bool initialised_ = false;
};

}  // namespace thunders::drivers

#endif  // THUNDERS_MOTOR_DRIVER_HPP
