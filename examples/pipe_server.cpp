// Minimal simulator server (agent47 PipeBridge only)

#include "flatsim/simulator.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> running{true};

static void on_sigint(int) { running.store(false); }

static datapod::Geo default_datum() { return datapod::Geo{51.98954034749562, 5.6584737410504715, 53.801823}; }

int main(int argc, char **argv) {
    std::signal(SIGINT, on_sigint);

    bool use_tcp = false;
    std::string host = "127.0.0.1";
    float width = 500.0f;
    float height = 500.0f;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--tcp") {
            use_tcp = true;
        } else if (a == "--ipc") {
            use_tcp = false;
        } else if (a == "--host" && i + 1 < argc) {
            host = argv[++i];
        } else if (a == "--width" && i + 1 < argc) {
            width = std::stof(argv[++i]);
        } else if (a == "--height" && i + 1 < argc) {
            height = std::stof(argv[++i]);
        }
    }

    simulator::Simulator sim(use_tcp ? simulator::Conn::TCP : simulator::Conn::IPC, host, width, height,
                             default_datum());
    std::cout << "[pipe_server] running (Ctrl+C to stop)\n";

    while (running.load()) {
        sim.tick(0.016f);
        sim.tock();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
}
