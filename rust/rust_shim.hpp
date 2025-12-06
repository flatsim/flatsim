// Header for Rust CXX bridge shim functions
#pragma once

#include "concord/concord.hpp"
#include <memory>

namespace flatsim {

    // Accessor functions for concord::Point
    double point_x(const concord::Point &p);
    double point_y(const concord::Point &p);

    // Factory function to create a point
    std::unique_ptr<concord::Point> create_point(double x, double y);

    // Calculate Euclidean distance between two points
    double calculate_distance(const concord::Point &p1, const concord::Point &p2);

} // namespace flatsim
