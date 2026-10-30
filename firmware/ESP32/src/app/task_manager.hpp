/**
 * @file task_manager.hpp
 * @brief FreeRTOS task spawner — owns the task handles and their lifecycle.
 *
 * Each subsystem exposes a `void run()` method that loops forever. The
 * TaskManager wraps that in a FreeRTOS task with the right priority / stack /
 * core affinity, and tracks the resulting `TaskHandle_t` so we can suspend,
 * resume, or delete tasks cleanly (e.g. during OTA or shutdown).
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_TASK_MANAGER_HPP
#define THUNDERS_TASK_MANAGER_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_config.hpp"

namespace thunders::app {

class RobotApp;

class TaskManager {
public:
    TaskManager();
    ~TaskManager();

    /// Spawn all subsystem tasks. Idempotent: safe to call multiple times.
    void spawn(RobotApp& app);

    /// Cancel every task and wait for them to exit (best-effort).
    void cancelAll();

private:
    // Task handle table. Adding a new task: add a slot here + a wrapper below.
    TaskHandle_t hControl_     = nullptr;
    TaskHandle_t hTelemetry_   = nullptr;
    TaskHandle_t hHealth_      = nullptr;
    TaskHandle_t hWatchdog_    = nullptr;
    TaskHandle_t hEstop_       = nullptr;
    TaskHandle_t hMqtt_        = nullptr;
    TaskHandle_t hSerial_     = nullptr;
    TaskHandle_t hImu_        = nullptr;
    TaskHandle_t hUltrasonic_ = nullptr;
    TaskHandle_t hBattery_    = nullptr;

    RobotApp* app_ = nullptr;

    // ─── Static wrappers (FreeRTOS entry points must be plain functions) ──
    static void controlTask(void* arg);
    static void telemetryTask(void* arg);
    static void healthTask(void* arg);
    static void watchdogTask(void* arg);
    static void estopTask(void* arg);
    static void mqttTask(void* arg);
    static void serialTask(void* arg);
    static void imuTask(void* arg);
    static void ultrasonicTask(void* arg);
    static void batteryTask(void* arg);

    /// Helper: create one task with standard error checking.
    bool createTask(TaskFunction_t fn, const char* name, std::uint32_t stackWords,
                    std::uint32_t prio, BaseType_t coreId, TaskHandle_t* out);
};

}  // namespace thunders::app

#endif  // THUNDERS_TASK_MANAGER_HPP
