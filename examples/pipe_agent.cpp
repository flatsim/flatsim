// Minimal agent client (agent47 PipeBridge only)
//
// Run:
//   ./build/.../pipe_server --ipc
//   ./build/.../pipe_agent --urdf examples_old/machines/urdf/tractor.urdf --name tractor_0

#include <agent47.hpp>
#include <agent47/bridge/pipe_bridge.hpp>
#include <agent47/model/urdf.hpp>

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

int main(int argc, char **argv) {
    std::string endpoint = default_endpoint();
    std::filesystem::path urdf = "examples_old/machines/urdf/tractor.urdf";
    std::string name = "pipe_agent";
    float vx = 0.0f;
    float wz = 0.0f;
    int seconds = 30;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--endpoint" && i + 1 < argc) {
            endpoint = argv[++i];
        } else if (a == "--urdf" && i + 1 < argc) {
            urdf = argv[++i];
        } else if (a == "--name" && i + 1 < argc) {
            name = argv[++i];
        } else if (a == "--vx" && i + 1 < argc) {
            vx = std::stof(argv[++i]);
        } else if (a == "--wz" && i + 1 < argc) {
            wz = std::stof(argv[++i]);
        } else if (a == "--seconds" && i + 1 < argc) {
            seconds = std::stoi(argv[++i]);
        }
    }

    dp::robot::Robot robot;
    robot.id.name = dp::String(name.c_str());
    robot.id.uuid = dp::sugar::uuid::generate_v4();
    robot.id.ip = dp::sugar::ip::v4(0, 0, 0, 0);
    robot.model = load_model_from_urdf(urdf);

    agent47::PipeBridge bridge;
    if (!bridge.connect(endpoint)) {
        std::cerr << "connect failed: " << endpoint << "\n";
        return 1;
    }
    if (!send_model_retry(bridge, robot, 5000)) {
        std::cerr << "model() failed\n";
        return 1;
    }
    std::cout << "[pipe_agent] connected\n";

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        if (seconds > 0) {
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= seconds) {
                break;
            }
        }

        (void)bridge.heartbeat(50);

        // Receive feedback (non-blocking)
        dp::Stamp<agent47::types::Feedback> fb;
        while (bridge.recv(fb, 0)) {
            std::cout << "fb x=" << fb.value.pose.point.x << " y=" << fb.value.pose.point.y << "\n";
        }

        // Drain sensors (non-blocking)
        agent47::types::SensorPacket pkt;
        while (bridge.sensor(pkt, 0)) {
            (void)pkt;
        }

        dp::Stamp<agent47::types::Command> cmd;
        cmd.timestamp = dp::Stamp<agent47::types::Command>::now();
        cmd.value.valid = true;
        cmd.value.twist.linear.vx = vx;
        cmd.value.twist.angular.vz = wz;
        bridge.send(cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}
