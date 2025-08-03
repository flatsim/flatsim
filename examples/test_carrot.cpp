#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== Carrot Algorithm Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("carrot_test", "space");
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

    // Test Carrot Algorithm with a straight path
    std::cout << "\n--- Testing Carrot Controller with Straight Path ---" << std::endl;

    // Set controller type to Carrot
    std::cout << "Setting controller to Carrot..." << std::endl;
    tractor.navcon->set_controller_type(navcon::NavconControllerType::CARROT);

    // Create a simple straight path - Carrot algorithm excels at simple goal-seeking behavior
    // Carrot algorithm moves directly toward the target point (like following a carrot on a stick)
    std::vector<concord::Point> straight_path = {
        {10.0f, 0.0f}, // First target
        {15.0f, 0.0f}, // Continue straight
        {25.0f, 0.0f}, // Keep going
        {35.0f, 0.0f}, // Straight line
        {45.0f, 0.0f}, // Simple path
        {55.0f, 0.0f}, // Final target
    };

    navcon::PathGoal path(straight_path, 3.0f, 3.0f, false); // Larger tolerance for simple algorithm

    std::cout << "Setting navigation path with " << straight_path.size() << " waypoints..." << std::endl;
    tractor.navcon->set_path(path);

    std::cout << "Starting Carrot Algorithm path following..." << std::endl;
    std::cout << "Carrot should demonstrate simple, direct movement toward each target" << std::endl;
    std::cout << "Watch for direct point-to-point navigation behavior" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    while (!tractor.navcon->is_path_completed()) {
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
            auto target = tractor.navcon->get_current_target();
            auto pos = tractor.get_position();

            // Calculate distance to current target
            float distance_to_target =
                std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

            std::cout << "Step " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw
                      << ", Distance to target=" << distance_to_target << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.navcon->is_path_completed()) {
        std::cout << "\n✅ Carrot Algorithm successfully completed the straight path!" << std::endl;
        std::cout << "Check Rerun visualization to see the direct point-to-point navigation." << std::endl;
    } else {
        std::cout << "\n❌ Carrot Algorithm did not complete the path within timeout." << std::endl;
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    // Additional test: Zigzag path to test Carrot's direct navigation behavior
    std::cout << "\n--- Testing Carrot with Zigzag Path Challenge ---" << std::endl;

    // Create a zigzag path to test how Carrot handles direction changes
    std::vector<concord::Point> zigzag_path = {
        {final_pos.point.x, final_pos.point.y},                 // Start from current position
        {final_pos.point.x + 10.0f, final_pos.point.y + 10.0f}, // Up-right
        {final_pos.point.x + 20.0f, final_pos.point.y - 5.0f},  // Down-right
        {final_pos.point.x + 30.0f, final_pos.point.y + 15.0f}, // Up-right again
        {final_pos.point.x + 40.0f, final_pos.point.y - 10.0f}, // Down-right again
        {final_pos.point.x + 50.0f, final_pos.point.y + 5.0f}   // Final up-right
    };

    navcon::PathGoal zigzag_goal(zigzag_path, 3.0f, 3.0f, false);
    tractor.navcon->set_path(zigzag_goal);

    std::cout << "Testing zigzag navigation with Carrot algorithm..." << std::endl;
    std::cout << "Carrot should show direct movement to each waypoint, creating sharp turns" << std::endl;

    start_time = std::chrono::steady_clock::now();
    step_count = 0;

    while (!tractor.navcon->is_path_completed()) {
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
            auto target = tractor.navcon->get_current_target();
            auto pos = tractor.get_position();
            float distance_to_target =
                std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));
            std::cout << "Zigzag Test " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw
                      << ", Distance=" << distance_to_target << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.navcon->is_path_completed()) {
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
