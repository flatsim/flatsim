#pragma once

#include "concord/concord.hpp"
#include <cmath>
#include <cstddef>

namespace fs {

    // Dynamic obstacle that moves with constant velocity
    // Activates when robot gets within activation_distance
    struct DynamicObstacle {
        size_t id = 0;
        concord::Point position;
        double vx = 0.0;                   // Velocity X (m/s)
        double vy = 0.0;                   // Velocity Y (m/s)
        double radius = 0.5;               // Obstacle radius (m)
        double uncertainty = 0.3;          // Position uncertainty std dev (m)
        double activation_distance = 10.0; // Distance to activate movement
        bool is_active = false;

        DynamicObstacle() = default;
        DynamicObstacle(size_t id, concord::Point pos, double vx, double vy, double r, double act_dist = 10.0,
                        double unc = 0.3)
            : id(id), position(pos), vx(vx), vy(vy), radius(r), uncertainty(unc), activation_distance(act_dist),
              is_active(false) {}

        // Update position based on robot proximity
        void update(double dt, double robot_x, double robot_y) {
            double dx = position.x - robot_x;
            double dy = position.y - robot_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < activation_distance) {
                is_active = true;
            }

            if (is_active) {
                position.x += vx * dt;
                position.y += vy * dt;
            }
        }

        // Predict future position at time t
        concord::Point predict(double t) const {
            if (!is_active) {
                return position;
            }
            return concord::Point{position.x + vx * t, position.y + vy * t, position.z};
        }
    };

} // namespace fs
