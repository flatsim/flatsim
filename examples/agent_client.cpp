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

    std::string machine_file = "examples/machines/tractor.json";
    concord::Pose spawn_pose(10.0, 10.0, 0.0);

    // Load machine configuration
    types::Machine machine_config = agent::Loader::load_from_json(machine_file, spawn_pose);
    machine_config.uuid = "agent_001";
    std::cout << "[Client] Loaded machine: " << machine_config.name << std::endl;

    // Create agent (rerun will be set up automatically after spawn)
    agent::Agent agent("");
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

    while (true) {
        // Set velocity commands (controls are automatically sent in tick())
        agent.set_velocity(0.5f, 0.6f); // Forward 0.5 m/s, turn 0.2 rad/s
        // BLOCKING: Wait for state update from simulator (tick blocks until message received)
        agent.tick(dt, 100); // 100ms timeout
        // Visualization (currently empty placeholder)
        // agent.tock();
    }

    // Despawn from simulator
    std::cout << "[Client] Despawning..." << std::endl;
    agent.despawn();

    std::cout << "[Client] Done" << std::endl;
    return 0;
}
