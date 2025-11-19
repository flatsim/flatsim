#include "flatsim/simulator.hpp"
#include "flatsim/robot/types.hpp"
#include <iostream>
#include <rerun.hpp>
#include <string>

int main(int argc, char *argv[]) {
    std::cout << "[Simulator] Starting..." << std::endl;

    // Parse command line arguments for server host
    std::string server_host = "0.0.0.0";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            server_host = argv[i + 1];
            ++i;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [--host SERVER_HOST]" << std::endl;
            std::cout << "  --host: Server host address for TCP connections" << std::endl;
            std::cout << "          0.0.0.0 = use client's connection IP (default)" << std::endl;
            std::cout << "          Specific IP = force that IP for all clients" << std::endl;
            return 0;
        }
    }

    std::cout << "[Simulator] Server host: " << server_host << std::endl;

    // Connect to Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "[Simulator] Failed to connect to rerun" << std::endl;
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // Set up world and simulator
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{10000.0f, 10000.0f, 300.0f};

    auto sim = std::make_shared<fs::Simulator>(rec);
    sim->init(world_datum, world_size);

    // Enable dispatcher for ZMQ process separation
    sim->enable_dispatcher(server_host);

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
