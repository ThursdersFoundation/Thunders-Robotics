// =============================================================================
// thunders/control/motor_controller.cpp
// -----------------------------------------------------------------------------
// Abstract base class for motor controllers.
//
// MotorController is the interface used by every higher-level controller
// (velocity, position, torque) to send outputs to a physical motor. Concrete
// subclasses wrap vendor-specific APIs (Talon FX, Spark Max, ODrive, etc.).
//
// The interface deliberately exposes both voltage and duty-cycle outputs:
// voltage mode is preferred when the bus voltage is well-regulated, while
// duty-cycle mode is preferred on brownout-prone buses where voltage mode
// would saturate.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>
#include <string>

namespace thunders::control {

class MotorController {
public:
    enum class ControlMode {
        DISABLED,    ///< Motor output forced to zero; brake mode active.
        PERCENT,     ///< Output in [-1, +1] of bus capacity.
        VOLTAGE,     ///< Output in volts.
        VELOCITY,    ///< Closed-loop velocity in rad/s.
        POSITION,    ///< Closed-loop position in radians.
        TORQUE       ///< Closed-loop torque in N*m.
    };

    MotorController() = default;
    virtual ~MotorController() = default;

    MotorController(const MotorController&)            = delete;
    MotorController& operator=(const MotorController&) = delete;

    /// Identity of this controller for logging.
    virtual const std::string& name() const noexcept = 0;

    /// Sets the control mode and command. The command's units depend on the
    /// mode: percent in [-1,1], voltage in volts, velocity in rad/s,
    /// position in radians, torque in N*m.
    virtual void set(ControlMode mode, double command) = 0;

    /// Convenience: sets percent output in [-1, +1].
    void set_percent(double v) { set(ControlMode::PERCENT, v); }

    /// Convenience: sets voltage output in volts.
    void set_voltage(double v) { set(ControlMode::VOLTAGE, v); }

    /// Stops the motor and engages the brake (if supported).
    virtual void stop() { set(ControlMode::DISABLED, 0.0); }

    // --- feedback --------------------------------------------------------
    virtual double position_radians() const = 0;
    virtual double velocity_rad_per_s() const = 0;
    virtual double torque_newton_meters() const = 0;
    virtual double bus_voltage() const = 0;
    virtual double output_current() const = 0;
    virtual double temperature_celsius() const = 0;

    // --- configuration ---------------------------------------------------
    virtual void set_inverted(bool inverted) = 0;
    virtual void set_brake_mode(bool brake) = 0;
    virtual void set_current_limit_amps(uint32_t limit) = 0;
    virtual void set_voltage_compensation(double volts) = 0;

    // --- fault reporting -------------------------------------------------
    struct Faults {
        bool hardware_fault      : 1;
        bool over_current        : 1;
        bool over_temperature    : 1;
        bool under_voltage       : 1;
        bool sensor_fault        : 1;
        bool communication_fault : 1;
    };
    virtual Faults get_faults() const = 0;
};

} // namespace thunders::control
