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
    std::cout << "=== MCA (Monte Carlo Approximation / DRA-MPPI) Risk-Aware Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("mca_test", "space");
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

    // Test MCA Controller with obstacle avoidance
    std::cout << "\n--- Testing MCA Controller with Dynamic Obstacles ---" << std::endl;
    std::cout << "MCA uses Monte Carlo sampling to estimate collision risk" << std::endl;
    std::cout << "This should show risk-aware trajectory planning with obstacle avoidance" << std::endl;

    // Set controller type to MCA
    std::cout << "Setting controller to MCA..." << std::endl;
    tractor.tracker->set_controller_type(drivekit::TrackerType::MCA);

    // Access the MCA controller directly to configure it
    auto mca_controller = dynamic_cast<drivekit::pred::MCAFollower *>(tractor.tracker->get_controller());
    if (mca_controller) {
        auto mca_config = mca_controller->get_mca_config();

        // Configure MCA parameters - tuned for smooth early avoidance
        // Lookahead distance = ref_velocity * dt * horizon_steps
        // We want ~15m lookahead for early smooth turns: 3.0 * 0.2 * 25 = 15m
        mca_config.horizon_steps = 25;       // Longer prediction horizon
        mca_config.dt = 0.2;                 // Time step (seconds)
        mca_config.num_samples = 400;        // More trajectory samples for smoother control
        mca_config.num_mc_samples = 2000;    // Monte Carlo samples
        mca_config.temperature = 0.3;        // Lower = more decisive/smoother
        mca_config.steering_noise = 0.4;     // Lower noise = smoother trajectories
        mca_config.acceleration_noise = 0.3; // Lower noise = smoother speed
        mca_config.ref_velocity = 3.0;       // Higher ref velocity for longer lookahead

        // Cost weights - tuned for smooth avoidance
        mca_config.weight_cte = 50.0;          // Lower = allow more deviation for smooth avoidance
        mca_config.weight_epsi = 50.0;         // Lower = allow heading changes
        mca_config.weight_vel = 1.0;           // Velocity tracking
        mca_config.weight_steering = 30.0;     // Higher = penalize sharp turns, prefer smooth curves
        mca_config.weight_acceleration = 10.0; // Higher = smoother speed changes

        // Risk-aware cost weights - start avoiding earlier
        mca_config.weight_soft_risk = 800.0; // High penalty encourages early avoidance
        mca_config.weight_hard_risk = 1e7;   // Very high hard constraint violation penalty
        mca_config.risk_threshold = 0.01;    // Very low threshold = very conservative (1%)

        // Velocity scaling when near obstacles
        mca_config.min_velocity_scale = 0.2; // Slow to 20% when high risk
        mca_config.risk_slowdown_gain = 6.0; // Moderate slowdown

        // Robot size safety margin (tractor is ~2.8m long, so add extra buffer)
        mca_config.robot_radius_margin = 1.0; // Extra 1.0m margin for early detection

        mca_controller->set_mca_config(mca_config);

        std::cout << "MCA Configuration:" << std::endl;
        std::cout << "  Horizon: " << mca_config.horizon_steps << " steps ("
                  << (mca_config.horizon_steps * mca_config.dt) << " seconds)" << std::endl;
        std::cout << "  Time step: " << mca_config.dt << " seconds" << std::endl;
        std::cout << "  MPPI samples: " << mca_config.num_samples << std::endl;
        std::cout << "  MC samples: " << mca_config.num_mc_samples << std::endl;
        std::cout << "  Risk threshold: " << (mca_config.risk_threshold * 100) << "%" << std::endl;
        std::cout << "  Reference velocity: " << mca_config.ref_velocity << std::endl;
    } else {
        std::cerr << "Failed to cast to MCA controller!" << std::endl;
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

    // Add obstacles to the world using the new obstacle system
    auto &world = simulator.get_world();

    // Add static obstacles (RED - don't move) - IN THE PATH
    world.add_obstacle(fs::StaticObstacle{1, concord::Point{15.0, 0.0}, 0.6, 0.1});  // In center of path
    world.add_obstacle(fs::StaticObstacle{2, concord::Point{30.0, 0.8}, 0.6, 0.1});  // Slightly off-center
    world.add_obstacle(fs::StaticObstacle{3, concord::Point{45.0, -0.8}, 0.6, 0.1}); // Other side

    // Add dynamic obstacles (GREEN - move when robot gets close)
    // Obstacle 1: Crosses path from below at x=20, activates when robot within 10m
    world.add_obstacle(fs::DynamicObstacle{1, concord::Point{20.0, -4.0}, 0.0, 0.6, 0.4, 10.0, 0.3});
    // Obstacle 2: Moves towards robot (head-on) at x=40, activates when robot within 15m
    world.add_obstacle(fs::DynamicObstacle{2, concord::Point{40.0, 0.0}, -0.8, 0.0, 0.4, 15.0, 0.3});
    // Obstacle 3: Crosses path from above at x=35, activates when robot within 10m
    world.add_obstacle(fs::DynamicObstacle{3, concord::Point{35.0, 4.0}, 0.0, -0.6, 0.4, 10.0, 0.3});

    std::cout << "\nObstacles added to world:" << std::endl;
    std::cout << "  Static obstacles: " << world.get_static_obstacles().size() << " (RED boxes)" << std::endl;
    std::cout << "  Dynamic obstacles: " << world.get_dynamic_obstacles().size() << " (GREEN boxes)" << std::endl;

    std::cout << "\nStarting MCA path following with obstacle avoidance..." << std::endl;
    std::cout << "Watch for:" << std::endl;
    std::cout << "  - Risk-aware trajectory planning" << std::endl;
    std::cout << "  - Dynamic obstacle prediction" << std::endl;
    std::cout << "  - Collision avoidance maneuvers" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.1f; // 10 Hz control loop (MCA is computationally heavier)

    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;

    while (!tractor.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        // Run simulation tick - this now handles obstacle updates and world constraints internally
        simulator.tick(dt);
        simulator.tock(1);

        // Track error statistics
        auto status = mca_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += status.cross_track_error;
        cte_samples++;

        // Print progress every 2 seconds
        if (step_count % 20 == 0) { // Every ~2 seconds at 10 Hz
            auto pos = tractor.get_position();
            std::cout << "Step " << step_count / 10 << "s: "
                      << "Robot(" << pos.point.x << "," << pos.point.y << "), "
                      << "Yaw=" << pos.angle.yaw << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "Heading Error=" << (status.heading_error * 180.0 / M_PI) << "deg" << std::endl;
        }

        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // ~10 Hz
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\nMCA successfully completed the path with obstacle avoidance!" << std::endl;
        std::cout << "Check Rerun visualization to see the risk-aware trajectory planning." << std::endl;
    } else {
        std::cout << "\nMCA did not complete the path within timeout." << std::endl;
    }

    std::cout << "\nStatistics:" << std::endl;
    std::cout << "  Total time: " << step_count / 10.0f << " seconds" << std::endl;
    std::cout << "  Max cross-track error: " << max_cte << " m" << std::endl;
    std::cout << "  Avg cross-track error: " << (cte_samples > 0 ? total_cte / cte_samples : 0.0f) << " m" << std::endl;

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    std::cout << "\nLegend:" << std::endl;
    std::cout << "  RED boxes = static obstacles (don't move)" << std::endl;
    std::cout << "  GREEN boxes = dynamic obstacles (moving)" << std::endl;

    return 0;
}
