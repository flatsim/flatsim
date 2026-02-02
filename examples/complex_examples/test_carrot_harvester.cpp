// Carrot Algorithm Path Following Test (Harvester)
//
// Run:
//   ./build/test_carrot_harvester

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <cmath>
#include <drivekit.hpp>
#include <iostream>
#include <thread>
#include <vector>

int main() {
    std::cout << "=== Carrot Algorithm Path Following Test (Harvester) ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    auto &harvester = sim.spawn_agent("machines/urdf/oxbo_harvester.urdf", utils::make_pose_2d(0.0, 0.0, 0.0f), "carrot_harvester_0");
    std::cout << "Harvester loaded: " << harvester.name() << " (" << harvester.uuid() << ")\n";

    harvester.tracker()->set_controller_type(drivekit::TrackerType::CARROT);
    harvester.set_tracker_enabled(true);

    auto params = harvester.tracker()->get_controller_params();
    params.carrot_distance = 1.5f;
    harvester.tracker()->set_controller_params(params);

    std::vector<datapod::Point> straight_path = {
        {15.0f, 0.0f}, {25.0f, 0.0f}, {40.0f, 0.0f}, {55.0f, 0.0f}, {70.0f, 0.0f}, {85.0f, 0.0f},
    };
    harvester.tracker()->set_path(drivekit::PathGoal(straight_path, 4.0f, 4.0f, false));

    auto start_time = std::chrono::steady_clock::now();
    const float dt = 0.016f;
    int step_count = 0;

    while (!harvester.tracker()->is_path_completed()) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > 120) {
            std::cout << "Timeout reached!\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        if (step_count % 120 == 0) {
            const auto target = harvester.tracker()->get_current_target();
            const auto pos = harvester.get_position();
            const float dx = target.x - static_cast<float>(pos.point.x);
            const float dy = target.y - static_cast<float>(pos.point.y);
            const float dist = std::sqrt(dx * dx + dy * dy);
            std::cout << (step_count / 60) << "s: Target(" << target.x << "," << target.y << "), Harvester("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << utils::get_yaw(pos) << ", Dist=" << dist
                      << "m\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    auto final_pos = harvester.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")\n";

    std::cout << "\n--- Testing Carrot with Zigzag Path Challenge ---" << std::endl;
    std::vector<datapod::Point> zigzag_path = {
        {static_cast<float>(final_pos.point.x), static_cast<float>(final_pos.point.y)},
        {static_cast<float>(final_pos.point.x + 15.0), static_cast<float>(final_pos.point.y + 15.0)},
        {static_cast<float>(final_pos.point.x + 30.0), static_cast<float>(final_pos.point.y - 10.0)},
        {static_cast<float>(final_pos.point.x + 45.0), static_cast<float>(final_pos.point.y + 20.0)},
        {static_cast<float>(final_pos.point.x + 60.0), static_cast<float>(final_pos.point.y - 15.0)},
        {static_cast<float>(final_pos.point.x + 75.0), static_cast<float>(final_pos.point.y + 10.0)},
    };
    harvester.tracker()->set_path(drivekit::PathGoal(zigzag_path, 4.0f, 4.0f, false));

    start_time = std::chrono::steady_clock::now();
    step_count = 0;

    while (!harvester.tracker()->is_path_completed()) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > 150) {
            std::cout << "Zigzag test timeout reached!\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        if (step_count % 60 == 0) {
            const auto target = harvester.tracker()->get_current_target();
            const auto pos = harvester.get_position();
            const float dx = target.x - static_cast<float>(pos.point.x);
            const float dy = target.y - static_cast<float>(pos.point.y);
            const float dist = std::sqrt(dx * dx + dy * dy);
            std::cout << "Zigzag " << (step_count / 60) << "s: Target(" << target.x << "," << target.y
                      << "), Harvester(" << pos.point.x << "," << pos.point.y << "), Dist=" << dist << "m\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== Carrot Harvester Test Complete ===" << std::endl;
    return 0;
}
