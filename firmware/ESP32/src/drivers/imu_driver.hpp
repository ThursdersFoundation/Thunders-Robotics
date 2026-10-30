/**
 * @file imu_driver.hpp
 * @brief BNO055 9-DOF IMU driver wrapper.
 *
 * Reads fused orientation (heading/pitch/roll) and calibration status from a
 * BNO055 sensor attached via I²C. When the sensor is absent or uncalibrated
 * the driver degrades gracefully — `sample()` returns zero and the health
 * monitor flags a warning.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_IMU_DRIVER_HPP
#define THUNDERS_IMU_DRIVER_HPP

#include <cstdint>

namespace thunders::drivers {

class ImuDriver {
public:
    ImuDriver();
    ~ImuDriver();

    /// Initialise I²C + BNO055 in NDOF fusion mode. Returns false if the chip
    /// is not responding on the I²C bus.
    bool begin();

    /// Read a fresh orientation sample. Safe to call from the IMU task.
    struct Orientation { float heading; float pitch; float roll; bool calibrated; };
    Orientation sample();

    /// Last known calibration status (0..3 per sensor). Returned in a struct
    /// so the caller can decide which axis to trust.
    struct Calib { std::uint8_t sys; std::uint8_t gyro; std::uint8_t accel; std::uint8_t mag; };
    Calib readCalib() const;

private:
    bool initialised_  = false;
    bool lastSampleOk_ = false;
};

}  // namespace thunders::drivers

#endif  // THUNDERS_IMU_DRIVER_HPP
