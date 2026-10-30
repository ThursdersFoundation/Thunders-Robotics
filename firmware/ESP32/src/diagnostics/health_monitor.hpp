/**
 * @file health_monitor.hpp
 * @brief Aggregates system health: battery, heap, CPU load, link status.
 *
 * Periodically samples the battery voltage (via ADC), free heap, uptime, and
 * feeds a structured summary to the logger + MQTT health topic. Also
 * evaluates safety-related thresholds (low battery, tip-over) and forwards
 * them to the e-stop supervisor when breached.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_HEALTH_MONITOR_HPP
#define THUNDERS_HEALTH_MONITOR_HPP

#include <cstdint>

#include "app/robot_app.hpp"

namespace thunders::diagnostics {

class HealthMonitor {
public:
    explicit HealthMonitor(app::RobotApp& app) : app_(app) {}
    ~HealthMonitor() = default;

    void begin();

    /// Called by the health task at kHealthMonitorPeriodMs.
    void update();

    /// Called by the battery task at its own rate.
    void sampleBattery();

    /// Latest battery voltage in volts.
    float batteryVoltage() const { return batteryV_; }

    /// Latest heap utilisation: free heap in bytes.
    std::uint32_t freeHeapBytes() const;

    /// Uptime in seconds.
    std::uint32_t uptimeSeconds() const;

private:
    app::RobotApp& app_;
    float         batteryV_    = 0.0f;
    std::uint32_t lowBattSinceMs_ = 0;
    bool          lowBattLatched_ = false;
};

}  // namespace thunders::diagnostics

#endif  // THUNDERS_HEALTH_MONITOR_HPP
