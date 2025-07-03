#include "polygon_drawer.hpp"
#include <iostream>

int main() {
    polygon_drawer::PolygonDrawer app;
    if (!app.start(8080)) {
        std::cerr << "Failed to start server" << std::endl;
        return 1;
    }

    std::cout << "Open http://localhost:8080 in your browser" << std::endl;
    std::cout << "Click points to draw a polygon, then click Done" << std::endl;

    // This blocks until user clicks Done
    auto points = app.collectPoints();

    std::cout << "\nReturned " << points.size() << " points:" << std::endl;
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "  [" << i << "]: " << points[i].lat << ", " << points[i].lon << std::endl;
    }

    return 0;
}
