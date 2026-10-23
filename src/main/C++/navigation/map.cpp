// =============================================================================
// thunders/navigation/map.cpp
// -----------------------------------------------------------------------------
// Static map of the robot's environment.
//
// A Map is a thin wrapper around an OccupancyGrid that adds metadata:
// the map's frame ID, its resolution (metres per cell), its origin in the
// world frame, and a list of named regions (e.g., "scoring_zone",
// "charging_station"). The OccupancyGrid holds the actual cell data; the
// Map provides a higher-level API used by the planner.
//
// Maps are loaded from disk (typically a YAML + PGM pair produced by a
// SLAM run) at startup and never modified at runtime.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace thunders::navigation {

class OccupancyGrid;

class Map {
public:
    struct Region {
        std::string name;
        double      min_x, min_y, max_x, max_y;
    };

    Map(std::shared_ptr<OccupancyGrid> grid,
        double resolution_m_per_cell,
        double origin_x_m,
        double origin_y_m,
        std::string frame_id)
        : grid_(std::move(grid)),
          resolution_m_per_cell_(resolution_m_per_cell),
          origin_x_m_(origin_x_m),
          origin_y_m_(origin_y_m),
          frame_id_(std::move(frame_id)) {}

    // --- accessors -------------------------------------------------------
    std::shared_ptr<OccupancyGrid> grid() const noexcept { return grid_; }
    double resolution_m_per_cell() const noexcept { return resolution_m_per_cell_; }
    double origin_x_m()            const noexcept { return origin_x_m_; }
    double origin_y_m()            const noexcept { return origin_y_m_; }
    const std::string& frame_id()  const noexcept { return frame_id_; }

    // --- coordinate conversions -----------------------------------------
    /// Converts a world (metres) coordinate to a grid cell index.
    /// Returns false if the world point is outside the map.
    bool world_to_cell(double world_x, double world_y,
                       int& cell_x, int& cell_y) const noexcept;

    /// Converts a grid cell index to a world coordinate (metres).
    void cell_to_world(int cell_x, int cell_y,
                       double& world_x, double& world_y) const noexcept;

    // --- regions ---------------------------------------------------------
    void add_region(Region r) { regions_[r.name] = std::move(r); }
    const Region* find_region(const std::string& name) const {
        auto it = regions_.find(name);
        return it == regions_.end() ? nullptr : &it->second;
    }
    /// True if the world point lies within the named region.
    bool in_region(const std::string& name, double x, double y) const;

private:
    std::shared_ptr<OccupancyGrid>              grid_;
    double                                      resolution_m_per_cell_;
    double                                      origin_x_m_;
    double                                      origin_y_m_;
    std::string                                 frame_id_;
    std::unordered_map<std::string, Region>     regions_;
};

} // namespace thunders::navigation
