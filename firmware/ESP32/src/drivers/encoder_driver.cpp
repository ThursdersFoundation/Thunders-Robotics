/**
 * @file encoder_driver.cpp
 * @brief PCNT-based quadrature encoder reader.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "drivers/encoder_driver.hpp"

#include <Arduino.h>

#include "diagnostics/logger.hpp"
#include "motor_config.hpp"
#include "pin_config.hpp"
#include "sensor_config.hpp"

// ESP32 PCNT peripheral driver (provided by Arduino-ESP32 core).
#include "driver/pcnt.h"

namespace thunders::drivers {

namespace {
constexpr pcnt_unit_t kUnitLeft  = PCNT_UNIT_0;
constexpr pcnt_unit_t kUnitRight = PCNT_UNIT_1;
constexpr const char* kTag = "[encoder]";

void configureUnit(pcnt_unit_t unit, std::uint8_t pin_a, std::uint8_t pin_b) {
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = pin_a;
    cfg.ctrl_gpio_num  = pin_b;
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.pos_mode       = PCNT_COUNT_INC;   // count on rising edges of A
    cfg.neg_mode       = PCNT_COUNT_DEC;   // decrement on falling edges
    cfg.counter_h_lim  =  16384;
    cfg.counter_l_lim  = -16384;
    cfg.unit           = unit;
    cfg.channel        = PCNT_CHANNEL_0;
    pcnt_unit_config(&cfg);

    pcnt_set_filter_value(unit, 100);  // 100 PCNT clock cycles glitch filter
    pcnt_filter_enable(unit);
    pcnt_counter_clear(unit);
    pcnt_count_mode_set_mode(unit, PCNT_CHANNEL_0,
                             PCNT_COUNT_INC, PCNT_COUNT_DEC,
                             PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    pcnt_counter_resume(unit);
}
}  // namespace

EncoderDriver::EncoderDriver()  = default;
EncoderDriver::~EncoderDriver() = default;

bool EncoderDriver::begin() {
    using namespace thunders::pins;
    configureUnit(kUnitLeft,  kEncoderLeftA,  kEncoderLeftB);
    configureUnit(kUnitRight, kEncoderRightA, kEncoderRightB);
    initialised_ = true;
    thunders::diagnostics::Logger::instance().info("%s ready (PCNT units 0/1)", kTag);
    return true;
}

EncoderDriver::Delta EncoderDriver::readDelta() {
    if (!initialised_) return {0, 0};
    int16_t left_now = 0, right_now = 0;
    pcnt_get_counter_value(kUnitLeft,  &left_now);
    pcnt_get_counter_value(kUnitRight, &right_now);

    Delta d{static_cast<std::int16_t>(left_now  - lastLeft_),
            static_cast<std::int16_t>(right_now - lastRight_)};
    lastLeft_  = left_now;
    lastRight_ = right_now;

    leftTotal_  += d.left;
    rightTotal_ += d.right;
    return d;
}

EncoderDriver::Counts EncoderDriver::readCounts() const {
    return {leftTotal_, rightTotal_};
}

EncoderDriver::Rpm EncoderDriver::readRpm(std::uint32_t dt_ms) {
    const Delta d = readDelta();
    if (dt_ms == 0) return {0.0f, 0.0f};
    constexpr float ms_per_min = 60'000.0f;
    const float scale = ms_per_min / static_cast<float>(dt_ms) /
                       thunders::config::motor::kCountsPerRevolution;
    return {d.left  * scale,
            d.right * scale};
}

}  // namespace thunders::drivers
