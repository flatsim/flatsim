#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/robot/types.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== Carrot Algorithm Path Following Test (Harvester) ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("carrot_harvester_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Create simulator
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    // Load harvester
    try {
        auto harvester_info =
            fs::Loader::load_from_json("examples/machines/oxbo_harvester.json",
                                       concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, 0.0f}});
        simulator.add_robot(harvester_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load harvester: " << e.what() << std::endl;
        return 1;
    }

    auto &harvester = simulator.get_robot(0);
    std::cout << "Harvester loaded: " << harvester.info.name << std::endl;

    // Test Carrot Algorithm with a straight path
    std::cout << "\n--- Testing Carrot Controller with Straight Path ---" << std::endl;

    // Set controller type to Carrot
    std::cout << "Setting controller to Carrot..." << std::endl;
    harvester.tracker->set_controller_type(drivekit::TrackerType::CARROT);

    // Set Carrot controller parameters (optional, using defaults)
    auto params = harvester.tracker->get_controller_params();
    params.carrot_distance = 1.5f; // Larger carrot distance for bigger machine
    harvester.tracker->set_controller_params(params);

    // Create a simple straight path with larger spacing for the bigger harvester
    // Harvester is ~7.68m long, so we use wider spacing between waypoints
    std::vector<concord::Point> straight_path = {
        {15.0f, 0.0f}, // First target - larger initial offset
        {25.0f, 0.0f}, // Continue straight
        {40.0f, 0.0f}, // Keep going
        {55.0f, 0.0f}, // Straight line
        {70.0f, 0.0f}, // Simple path
        {85.0f, 0.0f}, // Final target
    };

    drivekit::PathGoal path(straight_path, 4.0f, 4.0f, false); // Larger tolerance for bigger machine

    std::cout << "Setting navigation path with " << straight_path.size() << " waypoints..." << std::endl;
    harvester.tracker->set_path(path);

    std::cout << "Starting Carrot Algorithm path following..." << std::endl;
    std::cout << "Carrot should demonstrate simple, direct movement toward each target" << std::endl;
    std::cout << "Watch for direct point-to-point navigation behavior" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    while (!harvester.tracker->is_path_completed()) {
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
            auto target = harvester.tracker->get_current_target();
            auto pos = harvester.get_position();

            // Calculate distance to current target
            float distance_to_target =
                std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

            std::cout << "Step " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Harvester("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw
                      << ", Distance to target=" << distance_to_target << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (harvester.tracker->is_path_completed()) {
        std::cout << "\n✅ Carrot Algorithm successfully completed the straight path!" << std::endl;
        std::cout << "Check Rerun visualization to see the direct point-to-point navigation." << std::endl;
    } else {
        std::cout << "\n❌ Carrot Algorithm did not complete the path within timeout." << std::endl;
    }

    auto final_pos = harvester.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    // Additional test: Zigzag path to test Carrot's direct navigation behavior
    std::cout << "\n--- Testing Carrot with Zigzag Path Challenge ---" << std::endl;

    // Create a zigzag path with larger spacing for the harvester
    std::vector<concord::Point> zigzag_path = {
        {final_pos.point.x, final_pos.point.y},                 // Start from current position
        {final_pos.point.x + 15.0f, final_pos.point.y + 15.0f}, // Up-right
        {final_pos.point.x + 30.0f, final_pos.point.y - 10.0f}, // Down-right
        {final_pos.point.x + 45.0f, final_pos.point.y + 20.0f}, // Up-right again
        {final_pos.point.x + 60.0f, final_pos.point.y - 15.0f}, // Down-right again
        {final_pos.point.x + 75.0f, final_pos.point.y + 10.0f}  // Final up-right
    };

    drivekit::PathGoal zigzag_goal(zigzag_path, 4.0f, 4.0f, false);
    harvester.tracker->set_path(zigzag_goal);

    std::cout << "Testing zigzag navigation with Carrot algorithm..." << std::endl;
    std::cout << "Carrot should show direct movement to each waypoint, creating sharp turns" << std::endl;

    start_time = std::chrono::steady_clock::now();
    step_count = 0;

    while (!harvester.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 150) { // 1.5 minute timeout for zigzag test
            std::cout << "Zigzag test timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress for zigzag test
        if (step_count % 60 == 0) { // Every ~1 second
            auto target = harvester.tracker->get_current_target();
            auto pos = harvester.get_position();
            float distance_to_target =
                std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));
            std::cout << "Zigzag Test " << step_count / 60 << "s: Target(" << target.x << "," << target.y
                      << "), Harvester(" << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw
                      << ", Distance=" << distance_to_target << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (harvester.tracker->is_path_completed()) {
        std::cout << "\n✅ Carrot Algorithm successfully completed the zigzag test!" << std::endl;
        std::cout << "Carrot's direct navigation should create characteristic sharp direction changes." << std::endl;
    } else {
        std::cout << "\n❌ Carrot Algorithm did not complete zigzag test within timeout." << std::endl;
    }

    std::cout << "\n=== Carrot Algorithm Test Complete ===" << std::endl;
    std::cout << "Carrot algorithm characteristics observed:" << std::endl;
    std::cout << "- Simple, direct movement toward target points" << std::endl;
    std::cout << "- Sharp direction changes at waypoints" << std::endl;
    std::cout << "- Minimal computational overhead" << std::endl;
    std::cout << "- Best suited for simple point-to-point navigation" << std::endl;

    return 0;
}
