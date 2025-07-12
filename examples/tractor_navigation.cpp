#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/loader.hpp"
#include "flatsim/robot/controller.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== Tractor Navigation Example ===" << std::endl;
    std::cout << "This example demonstrates autonomous navigation using different control algorithms." << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("tractor_navigation", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Create simulator with default world
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{100.0f, 100.0f, 300.0f}; // Smaller world for navigation demo
    simulator.init(world_datum, world_size);

    // Load tractor configuration
    try {
        auto tractor_info =
            fs::Loader::load_from_json("examples/machines/tractor.json",
                                       concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, 0.0f}});

        // Add the tractor robot
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor configuration: " << e.what() << std::endl;
        return 1;
    }
    if (simulator.num_robots() == 0) {
        std::cerr << "Failed to add tractor robot!" << std::endl;
        return 1;
    }

    // Get reference to the tractor robot
    auto &tractor = simulator.get_robot(0);

    std::cout << "Tractor '" << tractor.info.name << "' loaded successfully!" << std::endl;

    // Set up navigation waypoints for the tractor to follow - further apart for testing
    std::vector<concord::Point> waypoints = {
        {10.0f, 10.0f}, // Go to first point
        {25.0f, 10.0f}, // Drive east
        {35.0f, 25.0f}, // Drive northeast
        {20.0f, 35.0f}, // Drive northwest
        {5.0f, 25.0f}   // Final point
    };

    // Demo different controller types
    std::vector<fs::ControllerType> controller_types = {fs::ControllerType::PID, fs::ControllerType::PURE_PURSUIT,
                                                        fs::ControllerType::STANLEY, fs::ControllerType::CARROT};

    std::vector<std::string> controller_names = {"PID Controller", "Pure Pursuit Controller", "Stanley Controller",
                                                 "Carrot Controller"};

    // Run simulation with different controllers
    for (size_t ctrl_idx = 0; ctrl_idx < controller_types.size(); ++ctrl_idx) {
        std::cout << "\n--- Testing " << controller_names[ctrl_idx] << " ---" << std::endl;

        // Set controller type
        tractor.set_navigation_controller_type(controller_types[ctrl_idx]);

        // Reset tractor to start position
        tractor.teleport(concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, 0.0f}});

        // Test point-to-point navigation first
        std::cout << "Point-to-point navigation: Going to (10, 10)" << std::endl;
        fs::NavigationGoal goal({10.0f, 10.0f}, 2.0f,
                                1.0f); // Target, tolerance, max_speed - realistic for slow tractor
        tractor.set_navigation_goal(goal);

        // Run simulation until goal is reached
        auto start_time = std::chrono::steady_clock::now();
        float dt = 0.016f; // 60 FPS for smoother simulation

        while (!tractor.is_navigation_goal_reached()) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

            if (elapsed > 30) { // 30 second timeout for closer target
                std::cout << "Goal not reached within timeout!" << std::endl;
                break;
            }

            simulator.tick(dt);
            simulator.tock(30);

            // Print progress every 3 seconds
            if (elapsed % 3 == 0) {
                auto pos = tractor.get_position();
                std::cout << "Goal: (10, 10), Robot: (" << pos.point.x << ", " << pos.point.y
                          << "), Yaw: " << pos.angle.yaw << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (tractor.is_navigation_goal_reached()) {
            std::cout << "Goal reached successfully!" << std::endl;
        }

        // Test path following
        std::cout << "Path following: Following waypoint path" << std::endl;
        fs::PathGoal path(waypoints, 2.0f, 2.0f, false); // waypoints, tolerance, max_speed, loop - realistic tolerance
        tractor.set_navigation_path(path);

        start_time = std::chrono::steady_clock::now();

        while (!tractor.is_navigation_path_completed()) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

            if (elapsed > 90) { // 90 second timeout for path (multiple waypoints)
                std::cout << "Path not completed within timeout!" << std::endl;
                break;
            }

            simulator.tick(dt);
            simulator.tock(5);

            // Print progress every 3 seconds
            if (elapsed % 3 == 0) {
                auto target = tractor.get_current_navigation_target();
                auto pos = tractor.get_position();
                std::cout << "Goal: (" << target.x << ", " << target.y << "), Robot: (" << pos.point.x << ", "
                          << pos.point.y << "), Yaw: " << pos.angle.yaw << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        if (tractor.is_navigation_path_completed()) {
            std::cout << "Path completed successfully!" << std::endl;
        }

        // Clear navigation and pause between tests
        tractor.clear_navigation_goal();
        tractor.clear_navigation_path();

        if (ctrl_idx < controller_types.size() - 1) {
            std::cout << "Press Enter to continue to next controller..." << std::endl;
            std::cin.get();
        }
    }

    std::cout << "\n=== Navigation Demo Complete ===" << std::endl;
    std::cout << "All controller types have been tested!" << std::endl;

    return 0;
}
