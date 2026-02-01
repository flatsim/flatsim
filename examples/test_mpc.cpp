// MPC Path Following Test (Agent API)
//
// Terminal 1 (IPC, default):
//   FLATSIM_IPC_DIR=./build/ipc ./build/linux/x86_64/release/simulator_server --ipc
// Terminal 2:
//   FLATSIM_IPC_DIR=./build/ipc ./build/linux/x86_64/release/test_mpc --ipc
//
// Or TCP:
//   ./build/linux/x86_64/release/simulator_server --tcp --host 127.0.0.1
//   ./build/linux/x86_64/release/test_mpc --host 127.0.0.1

#include "flatsim/agent.hpp"
#include "flatsim/agent/loader/loader.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <rerun.hpp>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
    std::cout << "=== MPC (Model Predictive Control) Path Following Test ===" << std::endl;

    // Connection:
    // - Default: IPC (`ipc://...`) using `FLATSIM_IPC_DIR` (defaults to `/tmp`).
    // - TCP: pass `--host 127.0.0.1` (server must be in TCP mode).
    std::string host;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            host = argv[++i];
        } else if (arg == "--tcp") {
            if (host.empty()) {
                host = "127.0.0.1";
            }
        } else if (arg == "--ipc") {
            host.clear();
        }
    }

    std::filesystem::path machine_file = "examples/machines/urdf/tractor.urdf";
    if (!std::filesystem::exists(machine_file)) {
        std::error_code ec;
        std::filesystem::path probe = std::filesystem::absolute(argv[0], ec).parent_path();
        if (!ec) {
            for (int up = 0; up < 8 && !probe.empty(); ++up) {
                auto candidate = probe / machine_file;
                if (std::filesystem::exists(candidate)) {
                    machine_file = candidate;
                    break;
                }
                probe = probe.parent_path();
            }
        }
    }

    // Load tractor - spawn at first waypoint
    // Spawn tractor at path start, pointing in +X direction (yaw=0)
    datapod::Pose spawn_pose =
        utils::make_pose_2d(0.0, 0.0, -1.5708f); // -90 deg to compensate for tractor's default orientation
    auto tractor_config = agent::Loader::load_from_urdf(machine_file, spawn_pose);
    tractor_config.uuid = "mpc_tractor";
    std::cout << "[Loader] Loaded: " << tractor_config.name << std::endl;

    // Create agent (rerun connection will be set up automatically after spawn)
    agent::Agent tractor(host);
    tractor.set_machine(tractor_config);

    // Spawn in simulator
    std::cout << "[Agent] Spawning in simulator..." << std::endl;
    if (!tractor.spawn()) {
        std::cerr << "[Agent] Failed to spawn. Is simulator_server running?" << std::endl;
        return 1;
    }
    std::cout << "[Agent] Spawned successfully!" << std::endl;

    // Switch to MPC controller (was initialized with PID by default)
    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MPC);
    tractor.controls().tracker().set_enabled(true);
    tractor.controls().set_navigation_enabled(true);

    std::cout << "\n--- Testing MPC Controller with S-Curve Path ---" << std::endl;

    // Access MPC follower to configure it
    auto mpc = dynamic_cast<drivekit::pred::MPCFollower *>(tractor.controls().tracker().tracker()->get_controller());
    if (mpc) {
        auto mpc_config = mpc->get_mpc_config();

        mpc_config.horizon_steps = 20;
        mpc_config.dt = 0.1;
        mpc_config.ref_velocity = 0.8;

        // Same weights as old test
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
    std::vector<datapod::Point> s_curve_waypoints = {
        {0.0f, 0.0f},   {5.0f, 0.0f},   {10.0f, 1.0f},  {15.0f, 3.0f},  {20.0f, 6.0f},  {25.0f, 10.0f}, {30.0f, 14.0f},
        {35.0f, 17.0f}, {40.0f, 19.0f}, {45.0f, 20.0f}, {50.0f, 19.0f}, {55.0f, 17.0f}, {60.0f, 14.0f}, {65.0f, 10.0f},
        {70.0f, 6.0f},  {75.0f, 3.0f},  {80.0f, 1.0f},  {85.0f, 0.0f},  {90.0f, 0.0f}};

    drivekit::PathGoal path(s_curve_waypoints, 2.0f, 2.0f, false);
    tractor.controls().tracker().tracker()->set_path(path);
    tractor.controls().tracker().tracker()->smoothen(25.0f); // 25cm intervals

    std::cout << "[MPC] Path set with " << s_curve_waypoints.size() << " waypoints" << std::endl;
    std::cout << "[MPC] Starting path following..." << std::endl;

    int step_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // Fixed 60 FPS like old test

    while (!tractor.controls().tracker().tracker()->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout
            std::cout << "[MPC] Timeout reached!" << std::endl;
            break;
        }

        // Tick agent (blocks until state received)
        // Navigation is automatic - controller updates inside tick()
        tractor.tick(dt, 100);

        // Tock for visualization
        tractor.tock();

        // Print progress every 2 seconds
        if (step_count % 120 == 0) {
            auto status = mpc->get_status();
            auto current_pose = tractor.machine().world_pose();

            std::cout << "[MPC] " << step_count / 60 << "s: " << "Pos(" << current_pose.point.x << ","
                      << current_pose.point.y << "), " << "Yaw=" << utils::get_yaw(current_pose) << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "HeadingErr=" << (status.heading_error * 180.0 / std::numbers::pi) << "deg" << std::endl;
        }

        step_count++;

        // Sleep to match 60 FPS like old test
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.controls().tracker().tracker()->is_path_completed()) {
        std::cout << "\n[MPC] Successfully completed S-curve path!" << std::endl;
        std::cout << "[MPC] Total time: " << step_count / 60.0f << " seconds" << std::endl;
    } else {
        std::cout << "\n[MPC] Did not complete path within timeout" << std::endl;
    }

    auto final_pose = tractor.machine().world_pose();
    std::cout << "[MPC] Final position: (" << final_pose.point.x << ", " << final_pose.point.y << ")" << std::endl;

    tractor.despawn();
    std::cout << "[Agent] Done" << std::endl;

    return 0;
}
