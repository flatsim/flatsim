#pragma once

#include "concord/concord.hpp"
#include <cstddef>

namespace fs {

    // Simple static obstacle that doesn't move
    struct StaticObstacle {
        size_t id = 0;
        concord::Point position;
        double radius = 0.5;      // Obstacle radius (m)
        double uncertainty = 0.1; // Position uncertainty std dev (m)

        StaticObstacle() = default;
        StaticObstacle(size_t id, concord::Point pos, double r, double unc = 0.1)
            : id(id), position(pos), radius(r), uncertainty(unc) {}
    };

} // namespace fs
