// Carrot Algorithm Path Following Test (LOCAL mode - single process)
//
// Migrated from `examples_old/test_carrot.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_carrot_local

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    std::cout << "=== Carrot Algorithm Path Following Test (LOCAL mode) ===" << std::endl;

    std::filesystem::path machine_file = "examples/machines/tractor.json";
    if (!std::filesystem::exists(machine_file)) {
        std::cerr << "[Error] Missing machine file: " << machine_file << std::endl;
        return 1;
    }

    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    auto &tractor = sim.spawn_agent(machine_file, concord::Pose(0.0, 0.0, 0.0f), "carrot_0");
    std::cout << "Tractor loaded: " << tractor.name() << " (" << tractor.uuid() << ")\n";

    std::cout << "\n--- Testing Carrot Controller with Straight Path ---" << std::endl;
    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::CARROT);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true);

    auto params = tractor.tracker()->get_controller_params();
    params.carrot_distance = 1.0f;
    tractor.tracker()->set_controller_params(params);

    std::vector<concord::Point> straight_path = {
        {10.0f, 0.0f}, {15.0f, 0.0f}, {25.0f, 0.0f}, {35.0f, 0.0f}, {45.0f, 0.0f}, {55.0f, 0.0f},
    };
    tractor.tracker()->set_path(drivekit::PathGoal(straight_path, 3.0f, 3.0f, false));

    auto start_time = std::chrono::steady_clock::now();
    const float dt = 0.016f;
    int step_count = 0;

    while (!tractor.tracker()->is_path_completed()) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                                              start_time)
                                 .count();
        if (elapsed > 120) {
            std::cout << "Timeout reached!\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        if (step_count % 120 == 0) {
            const auto target = tractor.tracker()->get_current_target();
            const auto pos = tractor.get_position();
            const float dx = target.x - static_cast<float>(pos.point.x);
            const float dy = target.y - static_cast<float>(pos.point.y);
            const float dist = std::sqrt(dx * dx + dy * dy);
            std::cout << (step_count / 60) << "s: Target(" << target.x << "," << target.y << "), Robot(" << pos.point.x
                      << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << ", Dist=" << dist << "m\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")\n";

    std::cout << "\n--- Testing Carrot with Zigzag Path Challenge ---" << std::endl;
    std::vector<concord::Point> zigzag_path = {
        {static_cast<float>(final_pos.point.x), static_cast<float>(final_pos.point.y)},
        {static_cast<float>(final_pos.point.x + 10.0), static_cast<float>(final_pos.point.y + 10.0)},
        {static_cast<float>(final_pos.point.x + 20.0), static_cast<float>(final_pos.point.y - 5.0)},
        {static_cast<float>(final_pos.point.x + 30.0), static_cast<float>(final_pos.point.y + 15.0)},
        {static_cast<float>(final_pos.point.x + 40.0), static_cast<float>(final_pos.point.y - 10.0)},
        {static_cast<float>(final_pos.point.x + 50.0), static_cast<float>(final_pos.point.y + 5.0)},
    };
    tractor.tracker()->set_path(drivekit::PathGoal(zigzag_path, 3.0f, 3.0f, false));

    start_time = std::chrono::steady_clock::now();
    step_count = 0;

    while (!tractor.tracker()->is_path_completed()) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                                              start_time)
                                 .count();
        if (elapsed > 150) {
            std::cout << "Zigzag test timeout reached!\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        if (step_count % 60 == 0) {
            const auto target = tractor.tracker()->get_current_target();
            const auto pos = tractor.get_position();
            const float dx = target.x - static_cast<float>(pos.point.x);
            const float dy = target.y - static_cast<float>(pos.point.y);
            const float dist = std::sqrt(dx * dx + dy * dy);
            std::cout << "Zigzag " << (step_count / 60) << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Dist=" << dist << "m\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== Carrot Algorithm Test Complete ===" << std::endl;
    return 0;
}

