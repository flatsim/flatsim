// SOC (SVG-MPPI) Stochastic Optimal Control Path Following Test (LOCAL mode)
//
// Migrated from `examples_old/test_soc.cpp` to the current Agent/Simulator APIs.
//
// Note:
// SOC's obstacle-aware cost uses `drivekit::WorldConstraints` (predicted obstacles). In the current Flatsim Agent API
// we don't yet pass WorldConstraints into `drivekit::Tracker::tick(...)`, so this example focuses on controller setup
// and path tracking while still placing obstacles in the physics world for visualization/collisions.
//
// Run:
//   ./build/linux/x86_64/release/test_soc_local

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <thread>
#include <vector>

static std::vector<datapod::Point> corridor_path(float x0, float x1, float step, float y = 0.0f) {
    std::vector<datapod::Point> path;
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
    std::cout << "=== SOC (SVG-MPPI) Path Following Test (LOCAL mode) ===" << std::endl;

    std::filesystem::path machine_file = "examples/machines/tractor.json";
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};

    simulator::Simulator sim(500.0f, 500.0f, datum);
    datapod::Pose spawn_pose = utils::make_pose_2d(0.0, 0.0, -static_cast<float>(std::numbers::pi / 2.0));
    auto &tractor = sim.spawn_agent(machine_file, spawn_pose);

    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::SOC);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true);

    auto *soc = dynamic_cast<drivekit::pred::SOCFollower *>(tractor.tracker()->get_controller());
    if (!soc) {
        std::cerr << "[Error] Failed to cast to SOC controller" << std::endl;
        return 1;
    }

    auto soc_config = soc->get_soc_config();
    soc_config.horizon_steps = 25;
    soc_config.dt = 0.2;
    soc_config.num_samples = 400;
    soc_config.guide_samples = 64;
    soc_config.temperature = 0.5;
    soc_config.guide_temperature = 0.5;
    soc_config.steering_noise = 0.4;
    soc_config.acceleration_noise = 0.3;
    soc_config.initial_steer_variance = 0.3;
    soc_config.ref_velocity = 3.0;

    soc_config.weight_cte = 50.0;
    soc_config.weight_epsi = 50.0;
    soc_config.weight_vel = 1.0;
    soc_config.weight_steering = 30.0;
    soc_config.weight_acceleration = 10.0;

    soc_config.svgd_iterations = 3;
    soc_config.svgd_step_size = 0.1;
    soc_config.kernel_bandwidth = 1.0;
    soc_config.use_covariance_adaptation = true;

    soc->set_soc_config(soc_config);

    std::cout << "[SOC] Config: horizon=" << soc_config.horizon_steps << " dt=" << soc_config.dt
              << " samples=" << soc_config.num_samples << " guide=" << soc_config.guide_samples
              << " svgd_it=" << soc_config.svgd_iterations << " ref_v=" << soc_config.ref_velocity << std::endl;

    const auto path_pts = corridor_path(0.0f, 50.0f, 0.5f, 0.0f);
    tractor.tracker()->set_path(drivekit::PathGoal(path_pts, 1.0f, 2.0f, false));

    // Obstacles (physics + visualization).
    sim.world().add_obstacle(types::StaticObstacle{1, datapod::Point{15.0, 0.0, 0.0}, 0.8, 0.1});
    sim.world().add_obstacle(types::StaticObstacle{2, datapod::Point{30.0, 0.8, 0.0}, 0.8, 0.1});
    sim.world().add_obstacle(types::StaticObstacle{3, datapod::Point{45.0, -0.8, 0.0}, 0.8, 0.1});

    sim.world().add_obstacle(types::DynamicObstacle{1, datapod::Point{20.0, -4.0, 0.0}, datapod::Point{0.0, 0.6, 0.0},
                                                    0.5, 0.3, 10.0, false});
    sim.world().add_obstacle(types::DynamicObstacle{2, datapod::Point{40.0, 0.0, 0.0}, datapod::Point{-0.8, 0.0, 0.0},
                                                    0.5, 0.3, 15.0, false});
    sim.world().add_obstacle(types::DynamicObstacle{3, datapod::Point{35.0, 4.0, 0.0}, datapod::Point{0.0, -0.6, 0.0},
                                                    0.5, 0.3, 10.0, false});

    std::cout << "[World] Static obstacles: " << sim.world().static_obstacles().size()
              << " | Dynamic obstacles: " << sim.world().dynamic_obstacles().size() << std::endl;

    float dt = 0.1f;
    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!tractor.tracker()->is_path_completed()) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 300) {
            std::cout << "[SOC] Timeout reached" << std::endl;
            break;
        }

        const auto pos = tractor.get_position();
        sim.world().update_obstacles(dt, pos.point.x, pos.point.y);

        sim.tick(dt);
        sim.tock();

        const auto status = soc->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        if (step_count % 20 == 0) {
            std::cout << "[SOC] " << (step_count / 10) << "s "
                      << "Pos(" << pos.point.x << "," << pos.point.y << ") "
                      << "CTE=" << status.cross_track_error << "m" << std::endl;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n=== SOC Results ===" << std::endl;
    std::cout << "Completed: " << (tractor.tracker()->is_path_completed() ? "yes" : "no") << std::endl;
    std::cout << "Total time: " << (step_count / 10.0f) << "s" << std::endl;
    std::cout << "Max CTE: " << max_cte << "m" << std::endl;
    std::cout << "Avg CTE: " << (cte_samples > 0 ? (total_cte / static_cast<float>(cte_samples)) : 0.0f) << "m"
              << std::endl;
    const auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
