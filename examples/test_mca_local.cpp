// MCA (Monte Carlo Approximation / DRA-MPPI) Risk-Aware Path Following Test (LOCAL mode)
//
// Migrated from `examples_old/test_mca.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_mca_local

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <thread>
#include <vector>

static std::vector<concord::Point> corridor_path(float x0, float x1, float step, float y = 0.0f) {
    std::vector<concord::Point> path;
    if (step <= 0.0f) {
        return path;
    }
    const int n = static_cast<int>((x1 - x0) / step) + 1;
    if (n > 0) {
        path.reserve(static_cast<size_t>(n));
    }
    for (float x = x0; x <= x1 + 1e-6f; x += step) {
        path.push_back({x, y});
    }
    return path;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    std::cout << "=== MCA (Monte Carlo Approximation / DRA-MPPI) Path Following Test (LOCAL mode) ===" << std::endl;

    std::filesystem::path machine_file = "examples/machines/tractor.json";
    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.801823};

    simulator::Simulator sim(500.0f, 500.0f, datum);
    concord::Pose spawn_pose(0.0, 0.0, -static_cast<float>(std::numbers::pi / 2.0));
    auto &tractor = sim.spawn_agent(machine_file, spawn_pose);

    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MCA);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true);

    auto *mca = dynamic_cast<drivekit::pred::MCAFollower *>(tractor.tracker()->get_controller());
    if (!mca) {
        std::cerr << "[Error] Failed to cast to MCA controller" << std::endl;
        return 1;
    }

    auto mca_config = mca->get_mca_config();
    mca_config.horizon_steps = 25;
    mca_config.dt = 0.2;
    mca_config.num_samples = 400;
    mca_config.num_mc_samples = 2000;
    mca_config.temperature = 0.3;
    mca_config.steering_noise = 0.4;
    mca_config.acceleration_noise = 0.3;
    mca_config.ref_velocity = 3.0;

    mca_config.weight_cte = 50.0;
    mca_config.weight_epsi = 50.0;
    mca_config.weight_vel = 1.0;
    mca_config.weight_steering = 30.0;
    mca_config.weight_acceleration = 10.0;

    mca_config.weight_soft_risk = 800.0;
    mca_config.weight_hard_risk = 1e7;
    mca_config.risk_threshold = 0.01;

    mca_config.min_velocity_scale = 0.2;
    mca_config.risk_slowdown_gain = 6.0;
    mca_config.robot_radius_margin = 1.0;
    mca->set_mca_config(mca_config);

    std::cout << "[MCA] Config: horizon=" << mca_config.horizon_steps << " dt=" << mca_config.dt
              << " samples=" << mca_config.num_samples << " mc=" << mca_config.num_mc_samples
              << " risk=" << (mca_config.risk_threshold * 100.0) << "%" << std::endl;

    // Corridor path (straight line)
    const auto path_pts = corridor_path(0.0f, 50.0f, 0.5f, 0.0f);
    drivekit::PathGoal path(path_pts, 1.0f, 2.0f, false);
    tractor.tracker()->set_path(path);

    // Obstacles (visual + physics); MCA obstacle avoidance requires passing WorldConstraints into drivekit::Tracker.
    sim.world().add_obstacle(types::StaticObstacle{1, concord::Point{15.0, 0.0}, 0.6, 0.1});
    sim.world().add_obstacle(types::StaticObstacle{2, concord::Point{30.0, 0.8}, 0.6, 0.1});
    sim.world().add_obstacle(types::StaticObstacle{3, concord::Point{45.0, -0.8}, 0.6, 0.1});

    sim.world().add_obstacle(
        types::DynamicObstacle{1, concord::Point{20.0, -4.0}, concord::Point{0.0, 0.6}, 0.4, 0.3, 10.0, false});
    sim.world().add_obstacle(
        types::DynamicObstacle{2, concord::Point{40.0, 0.0}, concord::Point{-0.8, 0.0}, 0.4, 0.3, 15.0, false});
    sim.world().add_obstacle(
        types::DynamicObstacle{3, concord::Point{35.0, 4.0}, concord::Point{0.0, -0.6}, 0.4, 0.3, 10.0, false});

    std::cout << "[World] Static obstacles: " << sim.world().static_obstacles().size()
              << " | Dynamic obstacles: " << sim.world().dynamic_obstacles().size() << std::endl;

    float dt = 0.1f; // 10 Hz (MCA is heavier)
    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!tractor.tracker()->is_path_completed()) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 300) {
            std::cout << "[MCA] Timeout reached" << std::endl;
            break;
        }

        // Update dynamic obstacles (not currently called from World::tick()).
        const auto pos = tractor.get_position();
        sim.world().update_obstacles(dt, pos.point.x, pos.point.y);

        sim.tick(dt);
        sim.tock();

        const auto status = mca->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        if (step_count % 20 == 0) {
            std::cout << "[MCA] " << (step_count / 10) << "s "
                      << "Pos(" << pos.point.x << "," << pos.point.y << ") "
                      << "Yaw=" << pos.angle.yaw << " "
                      << "CTE=" << status.cross_track_error << "m "
                      << "HeadErr=" << (status.heading_error * 180.0 / std::numbers::pi) << "deg" << std::endl;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const bool completed = tractor.tracker()->is_path_completed();
    std::cout << "\n=== MCA Results ===" << std::endl;
    std::cout << "Completed: " << (completed ? "yes" : "no") << std::endl;
    std::cout << "Total time: " << (step_count / 10.0f) << "s" << std::endl;
    std::cout << "Max CTE: " << max_cte << "m" << std::endl;
    std::cout << "Avg CTE: " << (cte_samples > 0 ? (total_cte / static_cast<float>(cte_samples)) : 0.0f) << "m"
              << std::endl;
    const auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}

