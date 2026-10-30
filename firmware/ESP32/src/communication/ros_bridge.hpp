/**
 * @file ros_bridge.hpp
 * @brief Optional rosbridge_suite WebSocket bridge.
 *
 * Provides a JSON-based bridge to a ROS 1/2 system running `rosbridge_suite`.
 * Messages are exchanged as JSON over a WebSocket; this class owns the
 * connection and translates `cmd_vel` / `odom` / `imu` / `scan` messages to
 * and from the RobotState. Marked experimental — disabled by default in
 * `app_config.hpp` (`kEnableRosBridge = false`).
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_ROS_BRIDGE_HPP
#define THUNDERS_ROS_BRIDGE_HPP

#include <cstdint>

namespace thunders::communication {

class RosBridge {
public:
    RosBridge();
    ~RosBridge();

    /// Set the broker URL (e.g. ws://192.168.1.50:9090). Returns false if
    /// disabled at compile time.
    bool begin(const char* url);

    /// Maintain the WebSocket connection + dispatch incoming `cmd_vel`.
    void loop();

    /// Publish an `odom` message. Caller supplies the raw JSON.
    void publishOdometry(const char* json);

    /// Publish an `imu` message.
    void publishImu(const char* json);

    /// Publish a `scan` message (from the ultrasonic).
    void publishScan(float distance_m);

    bool isConnected() const { return connected_; }

private:
    bool  initialised_ = false;
    bool  connected_  = false;
    char  host_[64];
    std::uint16_t port_ = 9090;
};

}  // namespace thunders::communication

#endif  // THUNDERS_ROS_BRIDGE_HPP
