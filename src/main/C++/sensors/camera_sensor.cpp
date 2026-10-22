// =============================================================================
// thunders/sensors/camera_sensor.cpp
// -----------------------------------------------------------------------------
// Camera (RGB or RGBD) abstraction.
//
// Wraps a USB camera, a CSI camera, or a depth camera (e.g., Intel RealSense
// D435, OAK-D). Exposes frames as raw byte buffers (BGR8 or YUYV) plus
// metadata (timestamp, exposure, gain, white balance). Decoding to RGB and
// running CV pipelines is the responsibility of higher-level code; this
// class is only the data source.
//
// The camera runs on a background thread that captures frames at the
// configured rate and pushes them into a triple-buffer. read() copies the
// newest frame out without blocking; if no new frame is available since
// the last read, read() returns false.
//
// Configuration:
//   - resolution: width x height (e.g., 640x480).
//   - fps: target frame rate (e.g., 30).
//   - format: pixel format (BGR8, YUYV, MJPEG).
//   - exposure: auto or manual (microseconds).
//   - gain: auto or manual (0-100).
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "thunders/sensors/sensor.cpp"

namespace thunders::sensors {

class CameraSensor : public Sensor {
public:
    enum class PixelFormat { BGR8, YUYV, MJPEG, Z16 };

    struct Frame {
        uint64_t                timestamp_ns;
        uint32_t                width;
        uint32_t                height;
        PixelFormat             format;
        std::vector<uint8_t>    data;
    };

    CameraSensor(std::string name,
                 std::string device_path,
                 uint32_t    width,
                 uint32_t    height,
                 double      fps,
                 PixelFormat format)
        : name_(std::move(name)),
          device_path_(std::move(device_path)),
          width_(width),
          height_(height),
          fps_(fps),
          format_(format) {}

    // --- Sensor interface ------------------------------------------------
    bool        init() override;
    bool        read() override;
    void        close() override;
    const std::string& name()              const noexcept override { return name_; }
    double      sample_rate_hz()           const noexcept override { return fps_; }
    bool        healthy()                  const noexcept override { return healthy_; }
    uint64_t    last_read_timestamp_ns()   const noexcept override { return last_ns_; }

    // --- camera-specific accessors --------------------------------------
    const Frame& latest_frame() const noexcept { return latest_; }

    /// Returns the most recent frame and clears the "new frame available"
    /// flag. Subsequent calls return false until a new frame is captured.
    bool take_frame(Frame& out);

    // --- configuration ---------------------------------------------------
    void set_exposure_us(int32_t us) { manual_exposure_us_ = us; auto_exposure_ = false; }
    void set_auto_exposure()         { auto_exposure_ = true; }
    void set_gain(double g)          { manual_gain_ = g; auto_gain_ = false; }
    void set_auto_gain()             { auto_gain_ = true; }
    void set_white_balance_k(uint32_t k) { manual_wb_k_ = k; auto_wb_ = false; }
    void set_auto_white_balance()         { auto_wb_ = true; }

    // --- accessors -------------------------------------------------------
    uint32_t    width()         const noexcept { return width_; }
    uint32_t    height()        const noexcept { return height_; }
    PixelFormat format()        const noexcept { return format_; }
    const std::string& device() const noexcept { return device_path_; }

private:
    std::string  name_;
    std::string  device_path_;
    uint32_t     width_;
    uint32_t     height_;
    double       fps_;
    PixelFormat  format_;
    bool         healthy_   {false};
    uint64_t     last_ns_   {0};

    Frame        latest_;
    std::mutex   frame_mutex_;
    std::atomic<bool> new_frame_available_ {false};

    // camera controls
    bool         auto_exposure_ {true};
    int32_t      manual_exposure_us_ {0};
    bool         auto_gain_     {true};
    double       manual_gain_   {0.0};
    bool         auto_wb_       {true};
    uint32_t     manual_wb_k_   {5500};
};

} // namespace thunders::sensors
