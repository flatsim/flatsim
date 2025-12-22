// MPC Path Following Test (LOCAL mode - single process)
//
// Run:
//   ./build/linux/x86_64/release/test_mpc_local
//
// This test runs the simulator and agent in the same process,
// eliminating IPC latency for tighter control loops.

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    std::cout << "=== MPC Path Following Test (LOCAL mode) ===" << std::endl;

    // Find machine file
    std::filesystem::path machine_file = "examples/machines/tractor.json";

    // GPS datum (reference point for local <-> WGS84 conversion)
    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.801823};

    // Create simulator in LOCAL mode (no IPC/TCP)
    simulator::Simulator sim(200, 200, datum); // 200x200 meter world
    std::cout << "[Simulator] Created in LOCAL mode" << std::endl;

    // Spawn tractor at path start
    concord::Pose spawn_pose(0.0, 0.0, -1.5708f); // -90 deg
    auto &tractor = sim.spawn_agent(machine_file.string(), spawn_pose);
    std::cout << "[Simulator] Spawned tractor: " << tractor.name() << std::endl;

    // Switch to MPC controller - using new convenience API
    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MPC);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true); // Shortcut!

    std::cout << "\n--- Testing MPC Controller with S-Curve Path ---" << std::endl;

    // Configure MPC - using new tracker() shortcut
    auto mpc = dynamic_cast<drivekit::pred::MPCFollower *>(tractor.tracker()->get_controller());
    if (mpc) {
        auto mpc_config = mpc->get_mpc_config();

        mpc_config.horizon_steps = 20;
        mpc_config.dt = 0.05;
        mpc_config.ref_velocity = 0.8;

        // Weights
        mpc_config.weight_cte = 200.0;
        mpc_config.weight_epsi = 150.0;
        mpc_config.weight_vel = 1.0;
        mpc_config.weight_steering = 50.0;
        mpc_config.weight_acceleration = 20.0;
        mpc_config.weight_steering_rate = 800.0;
        mpc_config.weight_acceleration_rate = 100.0;

        mpc->set_mpc_config(mpc_config);

        std::cout << "[MPC] Configuration:" << std::endl;
        std::cout << "  Horizon: " << mpc_config.horizon_steps << " steps" << std::endl;
        std::cout << "  Ref velocity: " << mpc_config.ref_velocity << " m/s" << std::endl;
    } else {
        std::cerr << "[Error] Failed to cast to MPC controller!" << std::endl;
        return 1;
    }

    // Create S-curve path
    std::vector<concord::Point> s_curve_waypoints = {
        {0.0f, 0.0f},   {5.0f, 0.0f},   {10.0f, 1.0f},  {15.0f, 3.0f},  {20.0f, 6.0f},  {25.0f, 10.0f}, {30.0f, 14.0f},
        {35.0f, 17.0f}, {40.0f, 19.0f}, {45.0f, 20.0f}, {50.0f, 19.0f}, {55.0f, 17.0f}, {60.0f, 14.0f}, {65.0f, 10.0f},
        {70.0f, 6.0f},  {75.0f, 3.0f},  {80.0f, 1.0f},  {85.0f, 0.0f},  {90.0f, 0.0f}};

    drivekit::PathGoal path(s_curve_waypoints, 2.0f, 2.0f, false);
    tractor.tracker()->set_path(path);  // Shortcut!
    tractor.tracker()->smoothen(25.0f); // Shortcut!

    std::cout << "[MPC] Path set with " << s_curve_waypoints.size() << " waypoints" << std::endl;
    std::cout << "[MPC] Starting path following..." << std::endl;

    int step_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    while (!tractor.tracker()->is_path_completed()) { // Shortcut!
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout
            std::cout << "[MPC] Timeout reached!" << std::endl;
            break;
        }

        // Single tick/tock - simulator handles everything in LOCAL mode
        sim.tick(dt);
        sim.tock();

        // Print progress every 2 seconds
        if (step_count % 120 == 0) {
            auto status = mpc->get_status();
            auto pos = tractor.get_position(); // Shortcut!

            std::cout << "[MPC] " << step_count / 60 << "s: "
                      << "Pos(" << pos.point.x << "," << pos.point.y << "), "
                      << "Yaw=" << pos.angle.yaw << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "HeadingErr=" << (status.heading_error * 180.0 / std::numbers::pi) << "deg" << std::endl;
        }

        step_count++;

        // Sleep to match 60 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.tracker()->is_path_completed()) { // Shortcut!
        std::cout << "\n[MPC] Successfully completed S-curve path!" << std::endl;
        std::cout << "[MPC] Total time: " << step_count / 60.0f << " seconds" << std::endl;
    } else {
        std::cout << "\n[MPC] Did not complete path within timeout" << std::endl;
    }

    auto final_pos = tractor.get_position(); // Shortcut!
    std::cout << "[MPC] Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    std::cout << "[Simulator] Done" << std::endl;

    return 0;
}
