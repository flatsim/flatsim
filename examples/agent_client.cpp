// Standalone agent client - connects to remote simulator server
// This process ONLY uses agent:: namespace code

#include "flatsim/agent.hpp"
#include "flatsim/agent/loader.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main(int argc, char **argv) {
    std::cout << "[Client] Starting agent client..." << std::endl;

    // Setup Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim_agent", "agent");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    // Load machine configuration
    std::string machine_file = "examples/machines/tractor.json";
    if (argc > 1) {
        machine_file = argv[1];
    }

    concord::Pose spawn_pose;
    spawn_pose.point.x = 0.0;
    spawn_pose.point.y = 0.0;
    spawn_pose.angle.yaw = 0.0;

    types::Machine machine_config = agent::Loader::load_from_json(machine_file, spawn_pose);
    machine_config.uuid = "agent_001";
    std::cout << "[Client] Loaded machine: " << machine_config.name << std::endl;

    // Create agent (connects to simulator via IPC) with Rerun visualization
    agent::Agent agent("", rec);
    agent.set_machine(machine_config);

    // Spawn in simulator
    std::cout << "[Client] Spawning in simulator..." << std::endl;
    if (!agent.spawn()) {
        std::cerr << "[Client] Failed to spawn in simulator" << std::endl;
        std::cerr << "[Client] Is the simulator server running?" << std::endl;
        return 1;
    }
    std::cout << "[Client] Spawned successfully!" << std::endl;

    // Simple control loop - drive forward in a circle
    std::cout << "[Client] Running control loop (10 seconds)..." << std::endl;
    const float dt = 0.016f;
    int iterations = 600; // ~10 seconds at 60 Hz

    for (int i = 0; i < iterations; ++i) {
        // Set velocity commands (controls are automatically sent in tick())
        agent.set_velocity(0.5f, 0.2f); // Forward 0.5 m/s, turn 0.2 rad/s

        // BLOCKING: Wait for state update from simulator (tick blocks until message received)
        agent.tick(dt, 100); // 100ms timeout

        // Visualization (currently empty placeholder)
        agent.tock();

        // Print pose every second
        if (i % 60 == 0) {
            auto pose = agent.machine().world_pose();
            std::cout << "[Client] Pose: (" << pose.point.x << ", " << pose.point.y << ") yaw=" << pose.angle.yaw
                      << std::endl;
        }
    }

    // Despawn from simulator
    std::cout << "[Client] Despawning..." << std::endl;
    agent.despawn();

    std::cout << "[Client] Done" << std::endl;
    return 0;
}
