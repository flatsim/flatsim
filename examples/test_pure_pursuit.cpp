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
    std::cout << "=== Pure Pursuit Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("pure_pursuit_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Create simulator
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    // Load tractor
    try {
        auto tractor_info =
            fs::Loader::load_from_json("examples/machines/tractor.json",
                                       concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, 0.0f}});
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;

    // Test Pure Pursuit with a curved path
    std::cout << "\n--- Testing Pure Pursuit Controller with Curved Path ---" << std::endl;
    
    // IMPORTANT: Set controller type BEFORE setting path
    std::cout << "Setting controller to Pure Pursuit..." << std::endl;
    tractor.set_navigation_controller_type(fs::ControllerType::PURE_PURSUIT);

    // Create a dense curved path with many waypoints for better Pure Pursuit performance
    std::vector<concord::Point> curved_path = {
        {5.0f, 0.0f},    // Start closer to robot
        {8.0f, 1.0f},    // Gradual start
        {12.0f, 3.0f},   // 
        {16.0f, 6.0f},   // 
        {20.0f, 10.0f},  // 
        {24.0f, 15.0f},  // 
        {28.0f, 21.0f},  // 
        {32.0f, 28.0f},  // 
        {35.0f, 35.0f},  // 
        {37.0f, 42.0f},  // 
        {38.0f, 49.0f},  // Top of curve
        {37.0f, 56.0f},  // Start turning back
        {35.0f, 62.0f},  // 
        {32.0f, 67.0f},  // 
        {28.0f, 71.0f},  // 
        {23.0f, 74.0f},  // 
        {18.0f, 76.0f},  // 
        {12.0f, 77.0f},  // 
        {6.0f, 76.0f},   // S-curve starts
        {1.0f, 74.0f},   // 
        {-3.0f, 71.0f},  // 
        {-6.0f, 67.0f},  // 
        {-8.0f, 62.0f},  // 
        {-9.0f, 56.0f},  // 
        {-8.0f, 50.0f},  // Final curve
        {-6.0f, 44.0f},  // 
        {-3.0f, 39.0f},  // 
        {1.0f, 35.0f},   // 
        {6.0f, 32.0f},   // 
        {12.0f, 30.0f}   // End point
    };

    fs::PathGoal path(curved_path, 2.5f, 3.0f, false); // Larger tolerance for the bigger path
    
    std::cout << "Setting navigation path with " << curved_path.size() << " waypoints..." << std::endl;
    tractor.set_navigation_path(path);

    std::cout << "Starting Pure Pursuit path following..." << std::endl;
    std::cout << "This should show smooth curved motion using lookahead points" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    while (!tractor.is_navigation_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 120) { // 2 minute timeout
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress every 2 seconds to see path following behavior
        if (step_count % 120 == 0) { // Every ~2 seconds at 60 FPS
            auto target = tractor.get_current_navigation_target();
            auto pos = tractor.get_position();
            std::cout << "Step " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.is_navigation_path_completed()) {
        std::cout << "\n✅ Pure Pursuit successfully completed the curved path!" << std::endl;
        std::cout << "Check Rerun visualization to see the smooth path following behavior." << std::endl;
    } else {
        std::cout << "\n❌ Pure Pursuit did not complete the path within timeout." << std::endl;
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
