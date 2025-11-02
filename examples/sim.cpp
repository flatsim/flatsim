#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include <iostream>
#include <rerun.hpp>

int main(int argc, char *argv[]) {
    std::cout << "[Simulator] Starting..." << std::endl;

    // Connect to Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "[Simulator] Failed to connect to rerun" << std::endl;
        return 1;
    }

    // Set up world and simulator
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{10000.0f, 10000.0f, 300.0f};

    auto sim = std::make_shared<fs::Simulator>(rec);
    sim->init(world_datum, world_size);

    // Enable dispatcher for ZMQ process separation
    sim->enable_dispatcher();

    std::cout << "[Simulator] Initialized. Waiting for robot connections..." << std::endl;
    std::cout << "[Simulator] Press Ctrl+C to exit" << std::endl;

    // Run simulator loop
    sim->ticktock(
        [&](float dt) {
            // No user logic needed - dispatcher handles everything
            return true; // Keep running
        },
        30); // 30 FPS visualization

    // Cleanup
    sim->disable_dispatcher();
    std::cout << "[Simulator] Shutting down..." << std::endl;

    return 0;
}
