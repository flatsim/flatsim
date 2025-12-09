#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/robot/types.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== Stanley Controller Path Following Test ===" << std::endl;

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

    // Load tractor - spawn at first waypoint pointing -90 degrees
    float initial_yaw = -1.5708f; // -90 degrees = -π/2 radians

    try {
        auto tractor_info = fs::Loader::load_from_json(
            "examples/machines/tractor.json",
            concord::Pose{concord::Point{5.0f, 0.0f}, concord::Euler{0.0f, 0.0f, initial_yaw}});
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;

    // Test Stanley Controller with a curved path
    std::cout << "\n--- Testing Stanley Controller with Curved Path ---" << std::endl;

    // Set controller type
    std::cout << "Setting controller to Stanley..." << std::endl;
    tractor.tracker->set_controller_type(drivekit::TrackerType::STANLEY);

    // Set Stanley controller parameters
    auto params = tractor.tracker->get_controller_params();
    params.cross_track_gain = 2.5f; // Cross-track error correction gain
    params.softening_gain = 1.5f;   // Heading error softening gain
    tractor.tracker->set_controller_params(params);

    // Create a curved path with waypoints
    // Stanley controller is good at minimizing cross-track error
    std::vector<concord::Point> curved_path = {
        {5.0f, 0.0f},   // Start closer to robot
        {8.0f, 1.0f},   // Gradual start
        {12.0f, 3.0f},  //
        {16.0f, 6.0f},  //
        {20.0f, 10.0f}, //
        {24.0f, 15.0f}, //
        {28.0f, 21.0f}, //
        {32.0f, 28.0f}, //
        {35.0f, 35.0f}, //
        {37.0f, 42.0f}, //
        {38.0f, 49.0f}, // Top of curve
        {37.0f, 56.0f}, // Start turning back
        {35.0f, 62.0f}, //
        {32.0f, 67.0f}, //
        {28.0f, 71.0f}, //
        {23.0f, 74.0f}, //
        {18.0f, 76.0f}, //
        {12.0f, 77.0f}, //
        {6.0f, 76.0f},  // S-curve starts
        {1.0f, 74.0f},  //
        {-3.0f, 71.0f}, //
        {-6.0f, 67.0f}, //
        {-8.0f, 62.0f}, //
        {-9.0f, 56.0f}, //
        {-8.0f, 50.0f}, // Final curve
        {-6.0f, 44.0f}, //
        {-3.0f, 39.0f}, //
        {1.0f, 35.0f},  //
        {6.0f, 32.0f},  //
        {12.0f, 30.0f}  // End point
    };

    drivekit::PathGoal path(curved_path, 2.5f, 3.0f, false); // Larger tolerance for the bigger path

    std::cout << "Setting navigation path with " << curved_path.size() << " waypoints..." << std::endl;
    tractor.tracker->set_path(path);

    // Smoothen the path for better Stanley performance
    std::cout << "Smoothening path with 50cm intervals..." << std::endl;
    tractor.tracker->smoothen(50.0f); // Add points every 50cm

    std::cout << "Starting Stanley path following..." << std::endl;
    std::cout << "This should show smooth path following with good cross-track error correction" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    while (!tractor.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 420) { // 2 minute timeout
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress every 2 seconds to see path following behavior
        if (step_count % 120 == 0) { // Every ~2 seconds at 60 FPS
            auto target = tractor.tracker->get_current_target();
            auto pos = tractor.get_position();
            std::cout << "Step " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\n✅ Stanley controller successfully completed the curved path!" << std::endl;
        std::cout << "Check Rerun visualization to see the cross-track error minimization." << std::endl;
    } else {
        std::cout << "\n❌ Stanley controller did not complete the path within timeout." << std::endl;
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
