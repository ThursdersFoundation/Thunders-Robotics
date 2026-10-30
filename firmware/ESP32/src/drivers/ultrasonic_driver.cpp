/**
 * @file ultrasonic_driver.cpp
 * @brief HC-SR04 driver implementation with median filtering.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "drivers/ultrasonic_driver.hpp"

#include <Arduino.h>
#include <algorithm>

#include "diagnostics/logger.hpp"
#include "pin_config.hpp"
#include "safety_config.hpp"
#include "sensor_config.hpp"

namespace thunders::drivers {

namespace {
constexpr const char* kTag = "[sonar]";

float medianOf5(float samples[5]) {
    std::sort(samples, samples + 5);
    return samples[2];
}
}  // namespace

UltrasonicDriver::UltrasonicDriver()  = default;
UltrasonicDriver::~UltrasonicDriver() = default;

bool UltrasonicDriver::begin() {
    using namespace thunders::pins;
    pinMode(kUltrasonicTrigger, OUTPUT);
    pinMode(kUltrasonicEcho,    INPUT);
    digitalWrite(kUltrasonicTrigger, LOW);
    initialised_ = true;
    thunders::diagnostics::Logger::instance().info("%s HC-SR04 on TRIG=%d ECHO=%d",
        kTag, kUltrasonicTrigger, kUltrasonicEcho);
    return true;
}

float UltrasonicDriver::sample() {
    using namespace thunders::config::sensor::ultrasonic;
    if (!initialised_) return kMaxDistanceM;

    using namespace thunders::pins;
    // Trigger pulse: 10 µs HIGH.
    digitalWrite(kUltrasonicTrigger, HIGH);
    delayMicroseconds(kTriggerPulseUs);
    digitalWrite(kUltrasonicTrigger, LOW);

    // Measure echo width with timeout.
    const std::uint32_t pulse_us =
        pulseIn(kUltrasonicEcho, HIGH, kEchoTimeoutUs);

    if (pulse_us == 0) {
        // No echo received within timeout → no obstacle (or sensor fault).
        lastReading_ = kMaxDistanceM;
        return lastReading_;
    }

    // Distance (m) = (pulse_us * speed_of_sound) / 2  (round-trip)
    float distance_m = (static_cast<float>(pulse_us) * kSpeedOfSoundMsUs) * 0.5f;
    if (distance_m > kMaxDistanceM) distance_m = kMaxDistanceM;
    if (distance_m < kMinDistanceM)  distance_m = kMinDistanceM;

    // Running median of last 5 samples.
    static float window[5] = {kMaxDistanceM, kMaxDistanceM,
                               kMaxDistanceM, kMaxDistanceM, kMaxDistanceM};
    static int idx = 0;
    window[idx] = distance_m;
    idx = (idx + 1) % kMedianWindowSize;
    float buf[5];
    std::copy(window, window + kMedianWindowSize, buf);
    lastReading_ = medianOf5(buf);
    return lastReading_;
}

bool UltrasonicDriver::obstacleNear() const {
    using namespace thunders::config::safety;
    return lastReading_ < kDangerDistanceM;
}

}  // namespace thunders::drivers
