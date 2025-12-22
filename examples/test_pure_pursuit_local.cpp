// Pure Pursuit Path Following Test (LOCAL mode - single process)
//
// Migrated from `examples_old/test_pure_pursuit.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_pure_pursuit_local

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    std::cout << "=== Pure Pursuit Path Following Test (LOCAL mode) ===" << std::endl;

    std::filesystem::path machine_file = "examples/machines/tractor.json";
    if (!std::filesystem::exists(machine_file)) {
        std::cerr << "[Error] Missing machine file: " << machine_file << std::endl;
        return 1;
    }

    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    concord::Pose spawn_pose(-5.0, -5.0, 0.0f);
    auto &tractor = sim.spawn_agent(machine_file, spawn_pose, "pure_pursuit_0");
    std::cout << "Tractor loaded: " << tractor.name() << " (" << tractor.uuid() << ")\n";

    std::cout << "\n--- Testing Pure Pursuit Controller with Curved Path ---" << std::endl;

    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::PURE_PURSUIT);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true);

    auto params = tractor.tracker()->get_controller_params();
    params.lookahead_distance = 3.0f;
    params.lookahead_gain = 1.0f;
    tractor.tracker()->set_controller_params(params);

    std::vector<concord::Point> curved_path = {
        {5.0f, 0.0f},   {8.0f, 1.0f},   {12.0f, 3.0f},  {16.0f, 6.0f},  {20.0f, 10.0f}, {24.0f, 15.0f},
        {28.0f, 21.0f}, {32.0f, 28.0f}, {35.0f, 35.0f}, {37.0f, 42.0f}, {38.0f, 49.0f}, {37.0f, 56.0f},
        {35.0f, 62.0f}, {32.0f, 67.0f}, {28.0f, 71.0f}, {23.0f, 74.0f}, {18.0f, 76.0f}, {12.0f, 77.0f},
        {6.0f, 76.0f},  {1.0f, 74.0f},  {-3.0f, 71.0f}, {-6.0f, 67.0f}, {-8.0f, 62.0f}, {-9.0f, 56.0f},
        {-8.0f, 50.0f}, {-6.0f, 44.0f}, {-3.0f, 39.0f}, {1.0f, 35.0f},  {6.0f, 32.0f},  {12.0f, 30.0f},
    };

    drivekit::PathGoal path(curved_path, 2.5f, 3.0f, false);
    tractor.tracker()->set_path(path);
    tractor.tracker()->smoothen(50.0f);

    std::cout << "Path set with " << curved_path.size() << " waypoints\n";
    std::cout << "Starting Pure Pursuit path following...\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    const float dt = 0.016f;
    int step_count = 0;

    while (!tractor.tracker()->is_path_completed()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed > 420) {
            std::cout << "Timeout reached!\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        if (step_count % 120 == 0) {
            auto target = tractor.tracker()->get_current_target();
            auto pos = tractor.get_position();
            std::cout << (step_count / 60) << "s: Target(" << target.x << "," << target.y << "), Robot(" << pos.point.x
                      << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << "\n";
        }
        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.tracker()->is_path_completed()) {
        std::cout << "\nPure Pursuit completed the curved path.\n";
    } else {
        std::cout << "\nPure Pursuit did not complete the path within timeout.\n";
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")\n";
    return 0;
}

