/**
 * @file task_manager.cpp
 * @brief FreeRTOS task spawning and lifecycle management.
 *
 * Each task function receives `this` (the TaskManager*) via `pvParameters`,
 * from which it can reach `app_` and therefore every sub-system on the
 * RobotApp. Each task runs a periodic loop pinned to its configured period.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "app/task_manager.hpp"

#include <Arduino.h>
#include <functional>

#include "app/robot_app.hpp"
#include "diagnostics/logger.hpp"
#include "sensor_config.hpp"

namespace thunders::app {

namespace {
constexpr const char* kTag = "[tasks]";

/// Run `body` every `kPeriodMs` ticks. Uses `vTaskDelayUntil` for jitter-free
/// timing — important for the 100 Hz control loop.
template <std::uint32_t kPeriodMs>
void periodicLoop(RobotApp* app, std::function<void(RobotApp&)> body) {
    const TickType_t delay = pdMS_TO_TICKS(kPeriodMs);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        if (app) body(*app);
        vTaskDelayUntil(&last, delay);
    }
}
}  // namespace

TaskManager::TaskManager()  = default;
TaskManager::~TaskManager() { cancelAll(); }

bool TaskManager::createTask(TaskFunction_t fn, const char* name,
                             std::uint32_t stackWords, std::uint32_t prio,
                             BaseType_t coreId, TaskHandle_t* out) {
    BaseType_t rc = xTaskCreatePinnedToCore(fn, name, stackWords, this,
                                           prio, out, coreId);
    if (rc != pdPASS) {
        diagnostics::Logger::instance().error("%s failed to create '%s'", kTag, name);
        return false;
    }
    diagnostics::Logger::instance().info(
        "%s created '%s' (stack=%u prio=%u core=%d)",
        kTag, name, stackWords, prio, static_cast<int>(coreId));
    return true;
}

void TaskManager::spawn(RobotApp& app) {
    app_ = &app;

    // Pin realtime tasks to core 1 (Arduino loop runs on core 0 by default).
    createTask(controlTask,    "ctrl",   config::kStackControl,     config::kPrioControl,     1, &hControl_);
    createTask(estopTask,      "estop",  config::kStackSafety,      config::kPrioSafety,      1, &hEstop_);
    createTask(watchdogTask,   "wdog",   config::kStackSafety,      config::kPrioSafety,      0, &hWatchdog_);

    // Communication & telemetry can live on core 0.
    createTask(telemetryTask,  "tele",   config::kStackApp,         config::kPrioTelemetry,   0, &hTelemetry_);
    createTask(healthTask,    "health", config::kStackDiagnostics, config::kPrioDiagnostics, 0, &hHealth_);
    createTask(mqttTask,      "mqtt",    config::kStackComms,       config::kPrioComms,       0, &hMqtt_);
    createTask(serialTask,    "serial", config::kStackApp,         config::kPrioComms,       0, &hSerial_);

    // Sensor sampling tasks — moderate priority, core 0.
    if (config::kEnableImu)
        createTask(imuTask, "imu", config::kStackDrivers, config::kPrioDrivers, 0, &hImu_);
    if (config::kEnableUltrasonic)
        createTask(ultrasonicTask, "sonar", config::kStackDrivers, config::kPrioDrivers, 0, &hUltrasonic_);
    createTask(batteryTask, "batt", config::kStackDrivers, config::kPrioDrivers, 0, &hBattery_);
}

void TaskManager::cancelAll() {
    auto kill = [](TaskHandle_t& h) {
        if (h) { vTaskDelete(h); h = nullptr; }
    };
    kill(hControl_);
    kill(hTelemetry_);
    kill(hHealth_);
    kill(hWatchdog_);
    kill(hEstop_);
    kill(hMqtt_);
    kill(hSerial_);
    kill(hImu_);
    kill(hUltrasonic_);
    kill(hBattery_);
}

// ────────────────────────────────────────────────────────────────────────────
// Task wrappers — each forwards to a per-subsystem update() on the RobotApp.
// ────────────────────────────────────────────────────────────────────────────
void TaskManager::controlTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::kControlLoopPeriodMs>(self->app_,
        [](RobotApp& a) { a.motorCtrl().update(); });
}

void TaskManager::telemetryTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::kTelemetryPeriodMs>(self->app_,
        [](RobotApp& /*a*/) {
            // Production: publish JSON state snapshot over MQTT + serial.
            // See communication/mqtt_client.cpp for the publisher skeleton.
        });
}

void TaskManager::healthTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::kHealthMonitorPeriodMs>(self->app_,
        [](RobotApp& a) { a.health().update(); });
}

void TaskManager::watchdogTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::kWatchdogTimeoutMs / 2>(self->app_,
        [](RobotApp& a) { a.watchdog().feed(); });
}

void TaskManager::estopTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::kTickPeriodMs>(self->app_,
        [](RobotApp& a) { a.estop().poll(); });
}

void TaskManager::mqttTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<100>(self->app_,
        [](RobotApp& a) { if (config::kEnableMqtt) a.mqtt().loop(); });
}

void TaskManager::serialTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<10>(self->app_,
        [](RobotApp& a) { a.serial().loop(); });
}

void TaskManager::imuTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::sensor::imu::kSamplePeriodMs>(self->app_,
        [](RobotApp& a) { a.imu().sample(); });
}

void TaskManager::ultrasonicTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::sensor::ultrasonic::kSamplePeriodMs>(self->app_,
        [](RobotApp& a) { a.ultrasonic().sample(); });
}

void TaskManager::batteryTask(void* arg) {
    auto* self = static_cast<TaskManager*>(arg);
    periodicLoop<config::sensor::battery::kSamplePeriodMs>(self->app_,
        [](RobotApp& a) { a.health().sampleBattery(); });
}

}  // namespace thunders::app
