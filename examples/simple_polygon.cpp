#include "polygon_drawer.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    polygon_drawer::PolygonDrawer app;
    if (!app.start(8080)) {
        std::cerr << "Failed to start server" << std::endl;
        return 1;
    }

    std::cout << "1. Test single point selection" << std::endl;
    std::cout << "Open http://localhost:8080 in your browser" << std::endl;
    std::cout << "Click to select a point, then click Done" << std::endl;

    auto point = app.collectSinglePoint();
    std::cout << "\nSelected point: " << point.lat << ", " << point.lon << std::endl;

    // Wait a bit for socket to be fully released
    std::cout << "\nWaiting for socket to be released..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Restart for polygon mode on different port
    if (!app.start(8081)) {
        std::cerr << "Failed to restart server" << std::endl;
        return 1;
    }

    std::cout << "\n2. Test polygon drawing" << std::endl;
    std::cout << "Open http://localhost:8081 in your browser" << std::endl;
    std::cout << "Click points to draw a polygon, then click Done" << std::endl;

    auto points = app.collectPoints();

    std::cout << "\nReturned " << points.size() << " points:" << std::endl;
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "  [" << i << "]: " << points[i].lat << ", " << points[i].lon << std::endl;
    }

    return 0;
}
