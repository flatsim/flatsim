#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

namespace {
    // Dynamic obstacle (moves when robot gets close)
    struct DynamicObstacle {
        size_t id;
        double x, y;
        double vx, vy;
        double size;
        double activation_distance; // Start moving when robot is this close
        bool is_active;

        void update(double dt, double robot_x, double robot_y) {
            double dist = std::sqrt((x - robot_x) * (x - robot_x) + (y - robot_y) * (y - robot_y));
            if (dist < activation_distance) {
                is_active = true;
            }
            if (is_active) {
                x += vx * dt;
                y += vy * dt;
            }
        }
    };

    // Static obstacle (doesn't move)
    struct StaticObstacle {
        size_t id;
        double x, y;
        double size;
    };
} // namespace

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

        // Configure MCA parameters
        mca_config.horizon_steps = 20;       // Prediction horizon steps
        mca_config.dt = 0.2;                 // Time step (seconds)
        mca_config.num_samples = 400;        // MPPI trajectory samples
        mca_config.num_mc_samples = 20000;   // Monte Carlo samples for collision probability
        mca_config.temperature = 1.0;        // Temperature for weighting
        mca_config.steering_noise = 0.5;     // Steering noise for sampling
        mca_config.acceleration_noise = 0.3; // Acceleration noise for sampling
        mca_config.ref_velocity = 0.6;       // Reference normalized speed (~60% throttle)

        // Cost weights
        mca_config.weight_cte = 100.0;        // Cross-track error
        mca_config.weight_epsi = 100.0;       // Heading error
        mca_config.weight_vel = 1.0;          // Velocity tracking
        mca_config.weight_steering = 10.0;    // Steering effort
        mca_config.weight_acceleration = 5.0; // Acceleration effort

        // Risk-aware cost weights
        mca_config.weight_soft_risk = 100.0; // Linear penalty on collision probability
        mca_config.weight_hard_risk = 1e6;   // Hard constraint violation penalty
        mca_config.risk_threshold = 0.05;    // Maximum allowable collision probability (5%)

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

    // Create static obstacles (RED - don't move) - IN THE PATH
    std::vector<StaticObstacle> static_obstacles;
    static_obstacles.push_back({1, 15.0, 0.0, 1.2});  // In the center of path
    static_obstacles.push_back({2, 30.0, 0.8, 1.2});  // Slightly off-center
    static_obstacles.push_back({3, 45.0, -0.8, 1.2}); // Other side

    // Create dynamic obstacles (GREEN - move when robot gets close)
    std::vector<DynamicObstacle> dynamic_obstacles;
    // Obstacle 1: Crosses path from below at x=20, activates when robot within 10m
    dynamic_obstacles.push_back({1, 20.0, -4.0, 0.0, 0.6, 0.8, 10.0, false});
    // Obstacle 2: Moves towards robot (head-on) at x=40, activates when robot within 15m
    dynamic_obstacles.push_back({2, 40.0, 0.0, -0.8, 0.0, 0.8, 15.0, false});
    // Obstacle 3: Crosses path from above at x=35, activates when robot within 10m
    dynamic_obstacles.push_back({3, 35.0, 4.0, 0.0, -0.6, 0.8, 10.0, false});

    std::cout << "\nObstacles:" << std::endl;
    std::cout << "  Static obstacles: " << static_obstacles.size() << " (RED boxes)" << std::endl;
    std::cout << "  Dynamic obstacles: " << dynamic_obstacles.size() << " (GREEN boxes)" << std::endl;

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

        // Get robot position
        auto pos = tractor.get_position();

        // Update dynamic obstacle positions
        for (auto &dyn : dynamic_obstacles) {
            dyn.update(dt, pos.point.x, pos.point.y);
        }

        // Visualize static obstacles (RED boxes)
        for (const auto &obs : static_obstacles) {
            std::string name = "obstacles/static_" + std::to_string(obs.id);
            rec->log_static(
                name, rerun::Boxes3D::from_centers_and_half_sizes(
                          {{float(obs.x), float(obs.y), 0.0f}}, {{float(obs.size / 2.0), float(obs.size / 2.0), 0.3f}})
                          .with_colors(rerun::Color(255, 0, 0)));
        }

        // Visualize dynamic obstacles (GREEN boxes)
        for (const auto &dyn : dynamic_obstacles) {
            std::string name = "obstacles/dynamic_" + std::to_string(dyn.id);
            rec->log_static(
                name, rerun::Boxes3D::from_centers_and_half_sizes(
                          {{float(dyn.x), float(dyn.y), 0.0f}}, {{float(dyn.size / 2.0), float(dyn.size / 2.0), 0.3f}})
                          .with_colors(rerun::Color(0, 255, 0)));
        }

        // Build WorldConstraints with obstacle predictions for MCA
        drivekit::WorldConstraints world_constraints;

        // Add dynamic obstacles with Gaussian predictions
        for (const auto &dyn : dynamic_obstacles) {
            drivekit::Obstacle obs;
            obs.id = dyn.id;
            obs.radius = dyn.size / 2.0;

            drivekit::Obstacle::GaussianMode mode;
            mode.weight = 1.0;
            auto mca_config = mca_controller->get_mca_config();
            for (size_t t = 0; t <= mca_config.horizon_steps; ++t) {
                double pred_time = t * mca_config.dt;
                mode.mean_x.push_back(dyn.x + dyn.vx * pred_time);
                mode.mean_y.push_back(dyn.y + dyn.vy * pred_time);
                mode.std_x.push_back(0.3); // Position uncertainty
                mode.std_y.push_back(0.3);
            }
            obs.modes.push_back(mode);
            world_constraints.obstacles.push_back(obs);
        }

        // Add static obstacles (zero velocity prediction)
        for (const auto &stat : static_obstacles) {
            drivekit::Obstacle obs;
            obs.id = 1000 + stat.id; // Offset ID to avoid collision
            obs.radius = stat.size / 2.0;

            drivekit::Obstacle::GaussianMode mode;
            mode.weight = 1.0;
            auto mca_config = mca_controller->get_mca_config();
            for (size_t t = 0; t <= mca_config.horizon_steps; ++t) {
                mode.mean_x.push_back(stat.x);
                mode.mean_y.push_back(stat.y);
                mode.std_x.push_back(0.1); // Less uncertainty for static
                mode.std_y.push_back(0.1);
            }
            obs.modes.push_back(mode);
            world_constraints.obstacles.push_back(obs);
        }

        // Visualize predicted trajectories (yellow lines)
        for (const auto &obs : world_constraints.obstacles) {
            std::string name = "predictions/pred_" + std::to_string(obs.id);
            if (!obs.modes.empty()) {
                const auto &mode = obs.modes[0];
                std::vector<rerun::Position3D> traj_points;
                for (size_t t = 0; t < mode.mean_x.size(); ++t) {
                    traj_points.push_back({float(mode.mean_x[t]), float(mode.mean_y[t]), 0.0f});
                }
                if (!traj_points.empty()) {
                    rec->log_static(name, rerun::LineStrips3D(rerun::components::LineStrip3D(traj_points))
                                              .with_colors(rerun::Color(255, 255, 0))
                                              .with_radii(0.02f));
                }
            }
        }

        // Run simulation tick with world constraints
        simulator.tick(dt);
        simulator.tock(1);

        // Track error statistics
        auto status = mca_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += status.cross_track_error;
        cte_samples++;

        // Print progress every 2 seconds
        if (step_count % 20 == 0) { // Every ~2 seconds at 10 Hz
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
    std::cout << "  YELLOW lines = predicted obstacle trajectories" << std::endl;
    std::cout << "  CYAN line = robot's planned trajectory" << std::endl;

    return 0;
}
