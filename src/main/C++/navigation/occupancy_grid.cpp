// =============================================================================
// thunders/navigation/occupancy_grid.cpp
// -----------------------------------------------------------------------------
// 2D occupancy grid for path planning.
//
// An OccupancyGrid is a 2D array of cells, each holding an occupancy
// probability in [0, 1] (0 = free, 1 = occupied). Cells with probability
// above the configured threshold are considered occupied for planning
// purposes.
//
// The grid is stored as a flat std::vector<float> in row-major order.
// Indexing is (cell_y * width_ + cell_x). Cells outside the grid are
// considered occupied (treated as walls) so the planner does not need to
// special-case the boundary.
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

namespace thunders::navigation {

class OccupancyGrid {
public:
    OccupancyGrid(uint32_t width, uint32_t height, float default_value = 0.0f)
        : width_(width), height_(height), cells_(width * height, default_value) {}

    // --- accessors -------------------------------------------------------
    uint32_t width()  const noexcept { return width_; }
    uint32_t height() const noexcept { return height_; }

    /// Returns the occupancy value at (x, y). Returns 1.0 (occupied) if the
    /// cell is outside the grid.
    float at(uint32_t x, uint32_t y) const noexcept {
        if (x >= width_ || y >= height_) return 1.0f;
        return cells_[y * width_ + x];
    }

    /// Sets the occupancy value at (x, y). No-op if out of bounds.
    void set(uint32_t x, uint32_t y, float v) {
        if (x >= width_ || y >= height_) return;
        cells_[y * width_ + x] = v;
    }

    /// True if the cell is considered occupied given the threshold.
    bool occupied(uint32_t x, uint32_t y, float threshold = 0.5f) const noexcept {
        return at(x, y) >= threshold;
    }

    /// True if the cell is in the "unknown" range (between low and high).
    bool unknown(uint32_t x, uint32_t y,
                 float low = 0.1f, float high = 0.9f) const noexcept {
        const float v = at(x, y);
        return v > low && v < high;
    }

    /// Clears every cell to the given value (default: free).
    void clear(float v = 0.0f) {
        for (auto& c : cells_) c = v;
    }

    /// Inflates every occupied cell by the given radius (in cells). Used to
    /// add a safety margin around obstacles so the planner does not scrape
    /// walls.
    void inflate(uint32_t radius_cells, float threshold = 0.5f) {
        // Naive O(N * radius^2) inflation; sufficient for small grids.
        std::vector<float> inflated = cells_;
        for (uint32_t y = 0; y < height_; ++y) {
            for (uint32_t x = 0; x < width_; ++x) {
                if (cells_[y * width_ + x] < threshold) continue;
                for (int dy = -int(radius_cells); dy <= int(radius_cells); ++dy) {
                    for (int dx = -int(radius_cells); dx <= int(radius_cells); ++dx) {
                        const int nx = int(x) + dx;
                        const int ny = int(y) + dy;
                        if (nx < 0 || ny < 0
                            || nx >= int(width_) || ny >= int(height_)) continue;
                        inflated[ny * width_ + nx] = 1.0f;
                    }
                }
            }
        }
        cells_ = std::move(inflated);
    }

private:
    uint32_t            width_;
    uint32_t            height_;
    std::vector<float>  cells_;
};

} // namespace thunders::navigation
