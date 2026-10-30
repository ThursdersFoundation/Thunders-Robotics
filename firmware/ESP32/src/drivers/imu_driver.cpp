/**
 * @file imu_driver.cpp
 * @brief BNO055 driver implementation using Adafruit_BNO055 library.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "drivers/imu_driver.hpp"

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "diagnostics/logger.hpp"
#include "pin_config.hpp"
#include "sensor_config.hpp"

namespace thunders::drivers {

namespace {
Adafruit_BNO055* bno = nullptr;  // owned here; constructed in begin()
constexpr const char* kTag = "[imu]";
}  // namespace

ImuDriver::ImuDriver()  = default;
ImuDriver::~ImuDriver() {
    delete bno;
    bno = nullptr;
}

bool ImuDriver::begin() {
    using namespace thunders::pins;
    using namespace thunders::config::sensor::imu;

    Wire.begin(kI2cSda, kI2cScl, thunders::board::kI2cFrequencyHz);

    bno = new Adafruit_BNO055(55, kI2cAddress, &Wire);
    if (!bno || !bno->begin()) {
        thunders::diagnostics::Logger::instance().error("%s BNO055 not found", kTag);
        delete bno;
        bno = nullptr;
        return false;
    }

    bno->setMode(static_cast<adafruit_bno055_opmode_t>(kOperationMode));
    delay(50);
    initialised_ = true;
    thunders::diagnostics::Logger::instance().info("%s BNO055 initialised (mode=0x%02X)",
                                                    kTag, kOperationMode);
    return true;
}

ImuDriver::Orientation ImuDriver::sample() {
    if (!initialised_ || !bno) return {0.0f, 0.0f, 0.0f, false};

    sensors_event_t ev;
    if (!bno->getEvent(&ev, Adafruit_BNO055::SENSOR_EULER)) {
        lastSampleOk_ = false;
        return {0.0f, 0.0f, 0.0f, false};
    }
    lastSampleOk_ = true;

    // BNO055 returns Euler angles in degrees. Convert to radians and apply
    // a low-pass EMA filter to suppress noise.
    using namespace thunders::config::sensor::imu;
    const float alpha = kFilterAlpha;
    const float rad = 3.14159265358979f / 180.0f;

    static float heading = 0.0f, pitch = 0.0f, roll = 0.0f;
    if (!lastSampleOk_) {
        heading = ev.orientation.x * rad;
        pitch   = ev.orientation.y * rad;
        roll    = ev.orientation.z * rad;
    } else {
        heading = alpha * (ev.orientation.x * rad) + (1.0f - alpha) * heading;
        pitch   = alpha * (ev.orientation.y * rad) + (1.0f - alpha) * pitch;
        roll    = alpha * (ev.orientation.z * rad) + (1.0f - alpha) * roll;
    }

    const std::uint8_t sys = bno->getCalibrationStatus(
        Adafruit_BNO055::CALIBRATION_STATUS_SYSTEM);
    const bool calibrated = (sys >= 2);

    return {heading, pitch, roll, calibrated};
}

ImuDriver::Calib ImuDriver::readCalib() const {
    if (!initialised_ || !bno) return {0, 0, 0, 0};
    std::uint8_t s, g, a, m;
    bno->getCalibration(&s, &g, &a, &m);
    return {s, g, a, m};
}

}  // namespace thunders::drivers
