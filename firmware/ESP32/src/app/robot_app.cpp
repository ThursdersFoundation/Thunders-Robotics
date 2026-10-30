/**
 * @file robot_app.cpp
 * @brief Implementation of the top-level robot application.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "app/robot_app.hpp"

#include "app_config.hpp"
#include "pin_config.hpp"

namespace thunders::app {

RobotApp::RobotApp()
    : state_(),
      motors_(std::make_unique<drivers::MotorDriver>()),
      encoders_(std::make_unique<drivers::EncoderDriver>()),
      imu_(std::make_unique<drivers::ImuDriver>()),
      ultrasonic_(std::make_unique<drivers::UltrasonicDriver>()),
      motor_ctrl_(std::make_unique<control::MotorController>(*motors_, *encoders_)),
      vel_ctrl_(std::make_unique<control::VelocityController>(*motor_ctrl_)),
      wifi_(std::make_unique<communication::WifiManager>()),
      mqtt_(std::make_unique<communication::MqttClient>()),
      serial_(std::make_unique<communication::SerialProtocol>()),
      watchdog_(std::make_unique<safety::Watchdog>()),
      estop_(std::make_unique<safety::EmergencyStop>(*motors_, *vel_ctrl_)),
      health_(std::make_unique<diagnostics::HealthMonitor>(*this)) {}

bool RobotApp::begin() {
    auto& log = diagnostics::Logger::instance();

    log.info("[app] Initialising drivers…");
    if (!motors_->begin())     { log.error("MotorDriver init failed");       return false; }
    if (!encoders_->begin())  { log.error("EncoderDriver init failed");     return false; }
    if (config::kEnableImu && !imu_->begin())
        { log.warn("ImuDriver init failed — continuing without IMU"); }
    if (config::kEnableUltrasonic && !ultrasonic_->begin())
        { log.warn("UltrasonicDriver init failed — continuing without sonar"); }

    log.info("[app] Initialising controllers…");
    motor_ctrl_->begin();
    vel_ctrl_->begin();

    log.info("[app] Initialising communication…");
    if (config::kEnableWifi) {
        wifi_->begin();
    }
    if (config::kEnableMqtt) {
        mqtt_->begin();
    }
    serial_->begin();

    log.info("[app] Initialising safety…");
    if (config::kEnableWatchdog) watchdog_->begin();
    estop_->begin();

    log.info("[app] Initialising diagnostics…");
    health_->begin();

    state_.setMode(config::RobotMode::kIdle);
    log.info("[app] RobotApp ready — entering IDLE");
    return true;
}

void RobotApp::start() {
    diagnostics::Logger::instance().info("[app] Spawning FreeRTOS task tree…");
    tasks_.spawn(*this);
}

void RobotApp::shutdown() {
    diagnostics::Logger::instance().info("[app] Shutdown requested — halting motors.");
    estop_->trigger(safety::EstopReason::kUserShutdown);
    tasks_.cancelAll();
}

}  // namespace thunders::app
