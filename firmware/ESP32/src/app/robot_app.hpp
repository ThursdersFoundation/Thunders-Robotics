/**
 * @file robot_app.hpp
 * @brief Top-level robot application — wires every sub-system together and
 *        owns the global RobotState.
 *
 * `RobotApp` is constructed once in `main.cpp`. It instantiates every driver,
 * controller, communication channel, safety supervisor, and diagnostic
 * monitor, then starts the FreeRTOS task tree via `TaskManager`.
 *
 * Single instance rule: there is one `RobotApp`, holding one `RobotState`.
 * Subsystems access the state by const reference (read) or via well-defined
 * setter methods (write).
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_ROBOT_APP_HPP
#define THUNDERS_ROBOT_APP_HPP

#include <memory>

#include "app/robot_state.hpp"
#include "app/task_manager.hpp"
#include "communication/mqtt_client.hpp"
#include "communication/serial_protocol.hpp"
#include "communication/wifi_manager.hpp"
#include "control/motor_controller.hpp"
#include "control/velocity_controller.hpp"
#include "diagnostics/health_monitor.hpp"
#include "diagnostics/logger.hpp"
#include "drivers/encoder_driver.hpp"
#include "drivers/imu_driver.hpp"
#include "drivers/motor_driver.hpp"
#include "drivers/ultrasonic_driver.hpp"
#include "safety/emergency_stop.hpp"
#include "safety/watchdog.hpp"

namespace thunders::app {

class RobotApp {
public:
    /// Construct sub-systems but DO NOT initialise hardware here. Hardware
    /// init happens in `begin()`.
    RobotApp();
    ~RobotApp() = default;

    RobotApp(const RobotApp&)            = delete;
    RobotApp& operator=(const RobotApp&) = delete;

    /// Initialise every sub-system. Returns false if any critical
    /// sub-system fails — caller should halt.
    [[nodiscard]] bool begin();

    /// Spawn all FreeRTOS tasks. Returns immediately (does not block).
    void start();

    /// Trigger a graceful shutdown (e.g. for OTA reflash).
    void shutdown();

    // ─── Accessors ─────────────────────────────────────────────────────────
    RobotState&                    state()       { return state_; }
    const RobotState&              state() const { return state_; }
    drivers::MotorDriver&          motors()      { return *motors_; }
    drivers::EncoderDriver&        encoders()    { return *encoders_; }
    drivers::ImuDriver&            imu()         { return *imu_; }
    drivers::UltrasonicDriver&      ultrasonic()  { return *ultrasonic_; }
    control::MotorController&      motorCtrl()   { return *motor_ctrl_; }
    control::VelocityController&   velocityCtrl(){ return *vel_ctrl_; }
    communication::WifiManager&    wifi()        { return *wifi_; }
    communication::MqttClient&     mqtt()        { return *mqtt_; }
    communication::SerialProtocol& serial()      { return *serial_; }
    safety::Watchdog&              watchdog()     { return *watchdog_; }
    safety::EmergencyStop&         estop()       { return *estop_; }
    diagnostics::HealthMonitor&    health()      { return *health_; }
    TaskManager&                   tasks()       { return tasks_; }

private:
    RobotState state_;

    std::unique_ptr<drivers::MotorDriver>        motors_;
    std::unique_ptr<drivers::EncoderDriver>      encoders_;
    std::unique_ptr<drivers::ImuDriver>          imu_;
    std::unique_ptr<drivers::UltrasonicDriver>  ultrasonic_;

    std::unique_ptr<control::MotorController>    motor_ctrl_;
    std::unique_ptr<control::VelocityController> vel_ctrl_;

    std::unique_ptr<communication::WifiManager>     wifi_;
    std::unique_ptr<communication::MqttClient>      mqtt_;
    std::unique_ptr<communication::SerialProtocol> serial_;

    std::unique_ptr<safety::Watchdog>     watchdog_;
    std::unique_ptr<safety::EmergencyStop> estop_;

    std::unique_ptr<diagnostics::HealthMonitor> health_;
    TaskManager                                  tasks_;
};

}  // namespace thunders::app

#endif  // THUNDERS_ROBOT_APP_HPP
