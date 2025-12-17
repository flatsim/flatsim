#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/world.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== SOC (SVG-MPPI) Stochastic Optimal Control Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("soc_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // Create simulator
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    // Load tractor - spawn at path start
    try {
        auto tractor_info = fs::Loader::load_from_json(
            "examples/machines/tractor.json",
            concord::Pose{
                concord::Point{0.0f, 0.0f},
                concord::Euler{0.0f, 0.0f, -1.5708f}}); // -90 deg to compensate for tractor's default orientation
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;

    // Test SOC Controller with obstacle avoidance
    std::cout << "\n--- Testing SOC (SVG-MPPI) Controller with Obstacles ---" << std::endl;
    std::cout << "SOC uses Stein Variational Gradient Descent to guide MPPI sampling" << std::endl;

    // Set controller type to SOC
    std::cout << "Setting controller to SOC..." << std::endl;
    tractor.tracker->set_controller_type(drivekit::TrackerType::SOC);

    // Access the SOC controller directly to configure it
    auto soc_controller = dynamic_cast<drivekit::pred::SOCFollower *>(tractor.tracker->get_controller());
    if (soc_controller) {
        auto soc_config = soc_controller->get_soc_config();

        // Configure SOC parameters for smooth obstacle avoidance
        // Lookahead = ref_velocity * dt * horizon_steps = 3.0 * 0.2 * 25 = 15m
        soc_config.horizon_steps = 25;
        soc_config.dt = 0.2;
        soc_config.num_samples = 400;
        soc_config.guide_samples = 64;
        soc_config.temperature = 0.5;
        soc_config.guide_temperature = 0.5;
        soc_config.steering_noise = 0.4;
        soc_config.acceleration_noise = 0.3;
        soc_config.initial_steer_variance = 0.3;
        soc_config.ref_velocity = 3.0; // Higher for longer lookahead

        // Cost weights - tuned for smooth avoidance
        soc_config.weight_cte = 50.0;  // Lower = allow deviation for avoidance
        soc_config.weight_epsi = 50.0; // Lower = allow heading changes
        soc_config.weight_vel = 1.0;
        soc_config.weight_steering = 30.0;     // Higher = smoother turns
        soc_config.weight_acceleration = 10.0; // Higher = smoother speed

        // SVGD parameters
        soc_config.svgd_iterations = 3;
        soc_config.svgd_step_size = 0.1;
        soc_config.kernel_bandwidth = 1.0;
        soc_config.use_covariance_adaptation = true;

        soc_controller->set_soc_config(soc_config);

        std::cout << "SOC Configuration:" << std::endl;
        std::cout << "  Horizon: " << soc_config.horizon_steps << " steps ("
                  << (soc_config.horizon_steps * soc_config.dt) << " seconds)" << std::endl;
        std::cout << "  Lookahead: ~" << (soc_config.ref_velocity * soc_config.dt * soc_config.horizon_steps)
                  << " meters" << std::endl;
        std::cout << "  MPPI samples: " << soc_config.num_samples << std::endl;
        std::cout << "  Guide samples: " << soc_config.guide_samples << std::endl;
        std::cout << "  SVGD iterations: " << soc_config.svgd_iterations << std::endl;
        std::cout << "  Reference velocity: " << soc_config.ref_velocity << std::endl;
    } else {
        std::cerr << "Failed to cast to SOC controller!" << std::endl;
        return 1;
    }

    // Create a corridor path (straight line with obstacles)
    std::vector<concord::Point> corridor_path;
    for (float x = 0.0f; x <= 50.0f; x += 0.5f) {
        corridor_path.push_back({x, 0.0f});
    }

    drivekit::PathGoal path(corridor_path, 1.0f, 2.0f, false);

    std::cout << "Setting navigation path with " << corridor_path.size() << " waypoints..." << std::endl;
    tractor.tracker->set_path(path);

    // Visualize reference path
    std::vector<rerun::Position3D> path_points;
    for (const auto &wp : corridor_path) {
        path_points.push_back({float(wp.x), float(wp.y), 0.0f});
    }
    rec->log_static("world/path", rerun::LineStrips3D(rerun::components::LineStrip3D(path_points))
                                      .with_colors(rerun::Color(150, 150, 150))
                                      .with_radii(0.05f));

    // Add obstacles to the world
    auto &world = simulator.get_world();

    // Add static obstacles (RED - don't move) - IN THE PATH
    world.add_obstacle(fs::StaticObstacle{1, concord::Point{15.0, 0.0}, 0.8, 0.1});
    world.add_obstacle(fs::StaticObstacle{2, concord::Point{30.0, 0.8}, 0.8, 0.1});
    world.add_obstacle(fs::StaticObstacle{3, concord::Point{45.0, -0.8}, 0.8, 0.1});

    // Add dynamic obstacles (GREEN - move when robot gets close)
    world.add_obstacle(fs::DynamicObstacle{1, concord::Point{20.0, -4.0}, 0.0, 0.6, 0.5, 10.0, 0.3});
    world.add_obstacle(fs::DynamicObstacle{2, concord::Point{40.0, 0.0}, -0.8, 0.0, 0.5, 15.0, 0.3});
    world.add_obstacle(fs::DynamicObstacle{3, concord::Point{35.0, 4.0}, 0.0, -0.6, 0.5, 10.0, 0.3});

    std::cout << "\nObstacles added to world:" << std::endl;
    std::cout << "  Static obstacles: " << world.get_static_obstacles().size() << " (RED)" << std::endl;
    std::cout << "  Dynamic obstacles: " << world.get_dynamic_obstacles().size() << " (GREEN)" << std::endl;

    std::cout << "\nStarting SOC path following with obstacle avoidance..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.1f;

    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;

    while (!tractor.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) {
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(1);

        auto status = soc_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += status.cross_track_error;
        cte_samples++;

        if (step_count % 20 == 0) {
            auto pos = tractor.get_position();
            std::cout << "Step " << step_count / 10 << "s: "
                      << "Robot(" << pos.point.x << "," << pos.point.y << "), "
                      << "CTE=" << status.cross_track_error << "m" << std::endl;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\nSOC successfully completed the path!" << std::endl;
    } else {
        std::cout << "\nSOC did not complete the path within timeout." << std::endl;
    }

    std::cout << "\nStatistics:" << std::endl;
    std::cout << "  Total time: " << step_count / 10.0f << " seconds" << std::endl;
    std::cout << "  Max cross-track error: " << max_cte << " m" << std::endl;
    std::cout << "  Avg cross-track error: " << (cte_samples > 0 ? total_cte / cte_samples : 0.0f) << " m" << std::endl;

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
