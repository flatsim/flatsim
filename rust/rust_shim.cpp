// C++ shim functions for Rust CXX bridge
#include "rust_shim.hpp"
#include "waypoint/types.hpp"
#include <cmath>

namespace flatsim {

    // Accessor functions for concord::Point
    double point_x(const concord::Point &p) { return p.x; }

    double point_y(const concord::Point &p) { return p.y; }

    // Create a concord::Point (returned as unique_ptr for CXX)
    std::unique_ptr<concord::Point> create_point(double x, double y) {
        return std::make_unique<concord::Point>(concord::Point{x, y});
    }

    // Calculate Euclidean distance between two points
    double calculate_distance(const concord::Point &p1, const concord::Point &p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

} // namespace flatsim
