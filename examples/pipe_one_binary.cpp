// One binary: simulator + agent, still communicating over IPC PipeBridge.

#include "flatsim/simulator.hpp"

#include <agent47.hpp>
#include <agent47/bridge/pipe_bridge.hpp>
#include <agent47/model/urdf.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

static std::filesystem::path ipc_dir() {
    const char *env = std::getenv("FLATSIM_IPC_DIR");
    std::filesystem::path dir = env && *env ? std::filesystem::path(env) : std::filesystem::path("/tmp");
    if (dir.is_relative()) {
        dir = std::filesystem::absolute(dir);
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir;
}

static std::string default_endpoint() { return std::string("ipc://") + (ipc_dir() / "agent47_peer.sock").string(); }

static dp::robot::Model load_model_from_urdf(const std::filesystem::path &urdf_path) {
    std::ifstream f(urdf_path);
    if (!f.good()) {
        throw std::runtime_error("failed to open urdf: " + urdf_path.string());
    }
    std::stringstream ss;
    ss << f.rdbuf();
    auto res = agent47::from_urdf_string(dp::String(ss.str().c_str()));
    if (res.is_err()) {
        throw std::runtime_error("failed to parse urdf");
    }
    return res.value();
}

static bool send_model_retry(agent47::PipeBridge &bridge, const dp::robot::Robot &robot, int total_ms) {
    const auto start = std::chrono::steady_clock::now();
    while (true) {
        if (bridge.model(robot, 250)) {
            return true;
        }
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
        if (elapsed.count() >= total_ms) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

int main() {
    const std::string endpoint = default_endpoint();
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};

    simulator::Simulator sim(simulator::Conn::IPC, "", 200.0f, 200.0f, datum);
    std::atomic<bool> running{true};

    std::thread server([&]() {
        while (running.load()) {
            sim.tick(0.016f);
            sim.tock();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    });

    std::thread agent([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        dp::robot::Robot robot;
        robot.id.name = dp::String("one_binary");
        robot.id.uuid = dp::sugar::uuid::generate_v4();
        robot.id.ip = dp::sugar::ip::v4(0, 0, 0, 0);
        robot.model = load_model_from_urdf("examples_old/machines/urdf/tractor.urdf");

        agent47::PipeBridge bridge;
        if (!bridge.connect(endpoint)) {
            std::cerr << "connect failed\n";
            running.store(false);
            return;
        }
        std::cout << "[pipe_one_binary] connected, sending model...\n";
        if (!send_model_retry(bridge, robot, 5000)) {
            std::cerr << "model failed\n";
            running.store(false);
            return;
        }
        std::cout << "[pipe_one_binary] model ok\n";

        const auto start = std::chrono::steady_clock::now();
        while (running.load()) {
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= 10) {
                running.store(false);
                break;
            }

            dp::Stamp<agent47::types::Command> cmd;
            cmd.timestamp = dp::Stamp<agent47::types::Command>::now();
            cmd.value.valid = true;
            cmd.value.twist.linear.vx = 0.5;
            cmd.value.twist.angular.vz = 0.0;
            bridge.send(cmd);

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    });

    agent.join();
    server.join();
    return 0;
}
