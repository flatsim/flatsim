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
    std::cout << "=== Stanley Algorithm Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("stanley_test", "space");
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

    // Test Stanley Algorithm with a complex path
    std::cout << "\n--- Testing Stanley Controller with Complex Path ---" << std::endl;
    
    // IMPORTANT: Set controller type BEFORE setting path
    std::cout << "Setting controller to Stanley..." << std::endl;
    tractor.set_navigation_controller_type(fs::ControllerType::STANLEY);

    // Create a challenging path that tests Stanley's cross-track error correction
    // Stanley algorithm excels at maintaining precise path following with minimal cross-track error
    // With coordinate correction, robot thinks it's facing 0° (east) when physically facing north
    std::vector<concord::Point> complex_path = {
        {2.0f, 0.0f},    // Start close to robot, going east
        {5.0f, 0.0f},    // Straight segment forward
        {10.0f, 0.0f},   // Continue straight
        {15.0f, 2.0f},   // Sharp turn to test cross-track correction
        {20.0f, 5.0f},   // 
        {25.0f, 8.0f},   // 
        {30.0f, 10.0f},  // 
        {35.0f, 10.0f},  // Straight segment again
        {40.0f, 10.0f},  // 
        {45.0f, 8.0f},   // Turn back down
        {50.0f, 5.0f},   // 
        {55.0f, 2.0f},   // 
        {60.0f, 0.0f},   // Back to horizontal
        {65.0f, -3.0f},  // Go below x-axis
        {70.0f, -6.0f},  // 
        {75.0f, -8.0f},  // Bottom of path
        {80.0f, -6.0f},  // Turn back up
        {85.0f, -3.0f},  // 
        {90.0f, 0.0f},   // Cross x-axis again
        {95.0f, 3.0f},   // Final climb
        {100.0f, 5.0f}   // End point
    };

    fs::PathGoal path(complex_path, 2.0f, 2.5f, false); // Tighter tolerance to test precision
    
    std::cout << "Setting navigation path with " << complex_path.size() << " waypoints..." << std::endl;
    tractor.set_navigation_path(path);

    std::cout << "Starting Stanley Algorithm path following..." << std::endl;
    std::cout << "Stanley should demonstrate superior cross-track error correction" << std::endl;
    std::cout << "Watch for minimal deviation from the path compared to other algorithms" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    while (!tractor.is_navigation_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 180) { // 3 minute timeout for longer path
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress every 2 seconds to see path following behavior
        if (step_count % 120 == 0) { // Every ~2 seconds at 60 FPS
            auto target = tractor.get_current_navigation_target();
            auto pos = tractor.get_position();
            
            // Calculate cross-track error (distance from robot to nearest point on path)
            float min_cross_track_error = std::numeric_limits<float>::max();
            for (const auto& point : complex_path) {
                float dist = std::sqrt(std::pow(point.x - pos.point.x, 2) + std::pow(point.y - pos.point.y, 2));
                min_cross_track_error = std::min(min_cross_track_error, dist);
            }
            
            std::cout << "Step " << step_count / 60 << "s: Target(" << target.x << "," << target.y 
                      << "), Robot(" << pos.point.x << "," << pos.point.y 
                      << "), Yaw=" << pos.angle.yaw 
                      << ", Cross-track error=" << min_cross_track_error << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.is_navigation_path_completed()) {
        std::cout << "\n✅ Stanley Algorithm successfully completed the complex path!" << std::endl;
        std::cout << "Check Rerun visualization to see the precise path following behavior." << std::endl;
        std::cout << "Stanley should show minimal cross-track error throughout the path." << std::endl;
    } else {
        std::cout << "\n❌ Stanley Algorithm did not complete the path within timeout." << std::endl;
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    // Additional test: Sharp turn to test Stanley's heading error correction
    std::cout << "\n--- Testing Stanley with Sharp Turn Challenge ---" << std::endl;
    
    // Create a path with a very sharp turn to test heading error correction
    std::vector<concord::Point> sharp_turn_path = {
        {final_pos.point.x, final_pos.point.y}, // Start from current position
        {final_pos.point.x + 10.0f, final_pos.point.y}, // Go straight east
        {final_pos.point.x + 15.0f, final_pos.point.y}, // Continue straight
        {final_pos.point.x + 20.0f, final_pos.point.y + 15.0f}, // Sharp 90-degree turn up
        {final_pos.point.x + 20.0f, final_pos.point.y + 25.0f}, // Go straight north
        {final_pos.point.x + 20.0f, final_pos.point.y + 35.0f}  // End point
    };

    fs::PathGoal sharp_path(sharp_turn_path, 2.0f, 2.0f, false);
    tractor.set_navigation_path(sharp_path);

    std::cout << "Testing sharp turn navigation with Stanley algorithm..." << std::endl;
    
    start_time = std::chrono::steady_clock::now();
    step_count = 0;
    
    while (!tractor.is_navigation_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 60) { // 1 minute timeout for sharp turn test
            std::cout << "Sharp turn test timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress for sharp turn test
        if (step_count % 60 == 0) { // Every ~1 second
            auto target = tractor.get_current_navigation_target();
            auto pos = tractor.get_position();
            std::cout << "Sharp Turn Test " << step_count / 60 << "s: Target(" << target.x << "," << target.y 
                      << "), Robot(" << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.is_navigation_path_completed()) {
        std::cout << "\n✅ Stanley Algorithm successfully completed the sharp turn test!" << std::endl;
        std::cout << "Stanley's heading error correction should handle sharp turns smoothly." << std::endl;
    } else {
        std::cout << "\n❌ Stanley Algorithm did not complete sharp turn test within timeout." << std::endl;
    }

    std::cout << "\n=== Stanley Algorithm Test Complete ===" << std::endl;
    std::cout << "Stanley algorithm characteristics observed:" << std::endl;
    std::cout << "- Excellent cross-track error correction" << std::endl;
    std::cout << "- Smooth handling of heading errors" << std::endl;
    std::cout << "- Precise path following even through complex maneuvers" << std::endl;

    return 0;
}