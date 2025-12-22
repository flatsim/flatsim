// MPPI (Model Predictive Path Integral) Path Following Test (LOCAL mode - single process)
//
// Migrated from `examples_old/test_mppi.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_mppi_local

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

    std::cout << "=== MPPI Path Following Test (LOCAL mode) ===" << std::endl;

    std::filesystem::path machine_file = "examples/machines/tractor.json";
    if (!std::filesystem::exists(machine_file)) {
        std::cerr << "[Error] Missing machine file: " << machine_file << std::endl;
        return 1;
    }

    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    auto &tractor = sim.spawn_agent(machine_file, concord::Pose(0.0, 0.0, -1.5708f), "mppi_0");
    std::cout << "Tractor loaded: " << tractor.name() << " (" << tractor.uuid() << ")\n";

    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MPPI);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true);

    auto *mppi = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker()->get_controller());
    if (!mppi) {
        std::cerr << "[Error] Failed to cast to MPPIFollower\n";
        return 1;
    }

    auto cfg = mppi->get_mppi_config();
    cfg.horizon_steps = 25;
    cfg.dt = 0.1;
    cfg.num_samples = 2000;
    cfg.temperature = 0.1;
    cfg.steering_noise = 0.15;
    cfg.acceleration_noise = 0.1;
    cfg.ref_velocity = 0.8;
    cfg.weight_cte = 200.0;
    cfg.weight_epsi = 180.0;
    cfg.weight_vel = 1.0;
    cfg.weight_steering = 80.0;
    cfg.weight_acceleration = 20.0;
    mppi->set_mppi_config(cfg);

    std::vector<concord::Point> s_curve_path = {
        {0.0f, 0.0f},   {5.0f, 0.0f},   {10.0f, 1.0f},  {15.0f, 3.0f},  {20.0f, 6.0f},  {25.0f, 10.0f},
        {30.0f, 14.0f}, {35.0f, 17.0f}, {40.0f, 19.0f}, {45.0f, 20.0f}, {50.0f, 19.0f}, {55.0f, 17.0f},
        {60.0f, 14.0f}, {65.0f, 10.0f}, {70.0f, 6.0f},  {75.0f, 3.0f},  {80.0f, 1.0f},  {85.0f, 0.0f},
        {90.0f, 0.0f},
    };

    tractor.tracker()->set_path(drivekit::PathGoal(s_curve_path, 2.0f, 2.0f, false));
    tractor.tracker()->smoothen(25.0f);

    auto start_time = std::chrono::steady_clock::now();
    const float dt = 0.016f;
    int step_count = 0;

    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;

    while (!tractor.tracker()->is_path_completed()) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                                              start_time)
                                 .count();
        if (elapsed > 300) {
            std::cout << "Timeout reached!\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        const auto status = mppi->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        if (step_count % 120 == 0) {
            const auto pos = tractor.get_position();
            std::cout << (step_count / 60) << "s: Robot(" << pos.point.x << "," << pos.point.y << "), "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "HeadingErr=" << (status.heading_error * 180.0 / M_PI) << "deg\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\nStatistics:\n";
    std::cout << "  Time: " << (step_count / 60.0f) << " s\n";
    std::cout << "  Max CTE: " << max_cte << " m\n";
    std::cout << "  Avg CTE: " << (cte_samples > 0 ? total_cte / cte_samples : 0.0f) << " m\n";

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")\n";
    return 0;
}

