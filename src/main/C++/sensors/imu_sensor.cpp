// =============================================================================
// thunders/sensors/imu_sensor.cpp
// -----------------------------------------------------------------------------
// 6-DOF Inertial Measurement Unit.
//
// Wraps a 3-axis gyro + 3-axis accelerometer (e.g., BMI270, MPU6050, or
// ADIS16470). Exposes a fused attitude estimate (roll, pitch, yaw) in
// addition to the raw gyro and accelerometer vectors.
//
// The fused attitude is computed by a complementary filter: high-pass the
// gyro (fast, drifts) and low-pass the accelerometer (slow, no drift) and
// combine them. The filter coefficient trades responsiveness against noise
// rejection; a typical value is 0.98 (98% gyro, 2% accel).
//
// Calibration: the IMU must be stationary and level during init() so the
// gyro bias and accelerometer gravity vector can be estimated. The bias
// is subtracted from every subsequent reading.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class IMUSensor : public Sensor {
public:
    struct Vector3 { double x, y, z; };
    struct Attitude { double roll, pitch, yaw; };

    IMUSensor(std::string name, uint8_t i2c_address, double sample_rate_hz)
        : name_(std::move(name)),
          i2c_address_(i2c_address),
          sample_rate_hz_(sample_rate_hz) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return sample_rate_hz_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- IMU-specific accessors -----------------------------------------
    const Vector3&   gyro_rad_per_s()   const noexcept { return gyro_; }
    const Vector3&   accel_mps2()       const noexcept { return accel_; }
    const Attitude&  attitude_rad()     const noexcept { return attitude_; }

    /// Resets the yaw to zero. Roll and pitch are derived from gravity
    /// and are not affected.
    void zero_yaw() { attitude_.yaw = 0.0; }

    /// Returns the temperature reported by the IMU, in degrees Celsius.
    double temperature_celsius() const noexcept { return temperature_c_; }

private:
    std::string  name_;
    uint8_t      i2c_address_;
    double       sample_rate_hz_;
    bool         healthy_   {false};
    uint64_t     last_ns_   {0};

    Vector3      gyro_      {0, 0, 0};
    Vector3      accel_     {0, 0, 0};
    Vector3      gyro_bias_ {0, 0, 0};
    Attitude     attitude_  {0, 0, 0};
    double       temperature_c_ {25.0};
    double       alpha_     {0.98};  ///< complementary filter coefficient
};

} // namespace thunders::sensors
