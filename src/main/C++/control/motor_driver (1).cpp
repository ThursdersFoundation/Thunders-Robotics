// =============================================================================
// thunders/control/motor_driver.cpp
// -----------------------------------------------------------------------------
// Low-level bridge between MotorController and a physical motor.
//
// MotorDriver is the lowest layer of the motor stack. It owns the vendor
// handle (e.g., a Talon FX CAN handle, a Spark Max serial handle) and
// translates calls from the MotorController interface into vendor-specific
// API calls.
//
// In production this file is replaced by per-vendor implementations
// (TalonFxMotorDriver, SparkMaxMotorDriver, etc.). The class declared here
// is a stub that records the last commanded output, useful for unit tests
// and simulation.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace thunders::control {

class MotorDriver {
public:
    explicit MotorDriver(std::string name, uint32_t can_id)
        : name_(std::move(name)), can_id_(can_id) {}

    const std::string& name()   const noexcept { return name_; }
    uint32_t           can_id() const noexcept { return can_id_; }

    // --- outputs ---------------------------------------------------------
    void set_percent_output(double v) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_percent_ = v;
    }
    void set_voltage_output(double v) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_voltage_ = v;
    }
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_percent_ = 0.0;
        last_voltage_ = 0.0;
    }

    // --- feedback (stubbed; real impl reads vendor API) ------------------
    double position_radians()      const { return 0.0; }
    double velocity_rad_per_s()    const { return 0.0; }
    double torque_newton_meters()  const { return 0.0; }
    double bus_voltage()           const { return 12.0; }
    double output_current()        const { return 0.0; }
    double temperature_celsius()   const { return 25.0; }

    // --- config ----------------------------------------------------------
    void set_inverted(bool v)            { inverted_ = v; }
    void set_brake_mode(bool v)          { brake_mode_ = v; }
    void set_current_limit_amps(uint32_t){ /* stub */ }
    void set_voltage_compensation(double){ /* stub */ }

    // --- inspection (used by tests) -------------------------------------
    double last_percent_output() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_percent_;
    }
    double last_voltage_output() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_voltage_;
    }
    bool   inverted()    const noexcept { return inverted_; }
    bool   brake_mode()  const noexcept { return brake_mode_; }

private:
    std::string         name_;
    uint32_t            can_id_;
    bool                inverted_   {false};
    bool                brake_mode_ {true};
    mutable std::mutex  mutex_;
    double              last_percent_ {0.0};
    double              last_voltage_ {0.0};
};

} // namespace thunders::control
