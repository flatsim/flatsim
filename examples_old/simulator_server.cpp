// Standalone simulator server - runs physics simulation and accepts agent connections
//
// IPC (default) uses `FLATSIM_IPC_DIR` (defaults to `/tmp`):
//   FLATSIM_IPC_DIR=./build/ipc ./build/linux/x86_64/release/simulator_server --ipc
//
// TCP:
//   ./build/linux/x86_64/release/simulator_server --tcp --host 127.0.0.1
// This process ONLY uses simulator:: namespace code

#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <signal.h>
#include <string>
#include <thread>

static bool running = true;

void signal_handler(int signal) {
    (void)signal;
    std::cout << "\n[Server] Shutting down..." << std::endl;
    running = false;
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);

    std::cout << "[Server] Starting simulator server..." << std::endl;
    std::cout << "[Server] Make sure to run 'rerun' in another terminal for visualization!" << std::endl;

    // Connection:
    // - Default: IPC (`ipc://...`) using `FLATSIM_IPC_DIR` (defaults to `/tmp`).
    // - TCP: pass `--tcp` (optional `--host <host>` to advertise to clients).
    bool use_tcp = false;
    std::string host = "127.0.0.1";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--tcp") {
            use_tcp = true;
        } else if (arg == "--ipc") {
            use_tcp = false;
        } else if (arg == "--host" && i + 1 < argc) {
            host = argv[++i];
        }
    }

    // Create simulator with IPC communication (rerun connects automatically)
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(use_tcp ? simulator::Conn::TCP : simulator::Conn::IPC, host, 500.0f, 500.0f, datum);

    std::cout << "[Server] Simulator ready. Waiting for agent connections..." << std::endl;
    std::cout << "[Server] Press Ctrl+C to stop" << std::endl;

    const float dt = 0.016f; // 60 Hz
    auto last_time = std::chrono::steady_clock::now();

    while (running) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

        if (elapsed >= 16) { // ~60 Hz
            try {
                // Tick simulator - processes spawn/despawn, controls, physics
                sim.tick(dt);
                sim.tock();
            } catch (const std::exception &e) {
                std::cerr << "[Server] Exception in tick: " << e.what() << std::endl;
            }

            last_time = current_time;
        }

        // Small sleep to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[Server] Simulator stopped" << std::endl;
    return 0;
}
