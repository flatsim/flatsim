#include "flatsim/loader.hpp"
#include "flatsim/robot/systems/client.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>

// Parse position argument "x,y,z"
concord::Point parse_position(const std::string &pos_str) {
    std::istringstream iss(pos_str);
    std::string token;
    std::vector<double> values;

    while (std::getline(iss, token, ',')) {
        values.push_back(std::stod(token));
    }

    if (values.size() != 3) {
        throw std::runtime_error("Position must be in format: x,y,z");
    }

    return concord::Point{values[0], values[1], values[2]};
}

// Parse color argument "r,g,b"
pigment::RGB parse_color(const std::string &color_str) {
    std::istringstream iss(color_str);
    std::string token;
    std::vector<int> values;

    while (std::getline(iss, token, ',')) {
        values.push_back(std::stoi(token));
    }

    if (values.size() != 3) {
        throw std::runtime_error("Color must be in format: r,g,b");
    }

    return pigment::RGB{static_cast<uint8_t>(values[0]), static_cast<uint8_t>(values[1]),
                        static_cast<uint8_t>(values[2])};
}

int main(int argc, char *argv[]) {
    // Parse command line arguments
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <filepath.json> <x,y,z> <r,g,b> [uuid]" << std::endl;
        std::cerr << "Example: " << argv[0] << " ../examples/machines/tractor.json 10,10,0 255,0,0" << std::endl;
        return 1;
    }

    std::string config_file = argv[1];
    std::string position_str = argv[2];
    std::string color_str = argv[3];
    std::string custom_uuid = (argc > 4) ? argv[4] : "";

    std::cout << "[Robot] Starting robot process..." << std::endl;
    std::cout << "[Robot] Config file: " << config_file << std::endl;
    std::cout << "[Robot] Position: " << position_str << std::endl;
    std::cout << "[Robot] Color: " << color_str << std::endl;

    try {
        // Parse CLI arguments
        concord::Point position = parse_position(position_str);
        pigment::RGB color = parse_color(color_str);

        // Load RobotInfo from JSON file
        std::filesystem::path config_path(config_file);
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "[Robot] Config file not found: " << config_file << std::endl;
            return 1;
        }

        // Load robot info with overrides
        concord::Pose spawn_pose(position.x, position.y, position.z);
        fs::RobotInfo robot_info =
            fs::Loader::load_from_json(config_path, spawn_pose, "", color); // Empty name, will use seqid

        // Override UUID if provided
        if (!custom_uuid.empty()) {
            robot_info.uuid = custom_uuid;
        }

        std::cout << "[Robot] Loaded robot: " << robot_info.type << " (UUID: " << robot_info.uuid << ")" << std::endl;

        // Create ZMQ client
        fs::Client client;
        if (!client.init()) {
            std::cerr << "[Robot] Failed to initialize client" << std::endl;
            return 1;
        }

        // Spawn robot in simulator
        std::cout << "[Robot] Spawning robot in simulator..." << std::endl;
        if (!client.spawn_robot(robot_info)) {
            std::cerr << "[Robot] Failed to spawn robot" << std::endl;
            return 1;
        }

        std::cout << "[Robot] Robot spawned successfully!" << std::endl;
        std::cout << "[Robot] Running control loop (press Ctrl+C to exit)..." << std::endl;

        // Simple control loop
        auto last_time = std::chrono::steady_clock::now();
        float steering = 0.0f;
        float throttle = 0.0f;

        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = now - last_time;
            last_time = now;

            // Receive physics state from simulator
            auto physics_state = client.receive_physics_state();
            if (physics_state.has_value()) {
                // We received updated physics state
                // std::cout << "[Robot] Position: " << physics_state->pose.point.x << ", "
                //          << physics_state->pose.point.y << std::endl;
            }

            // Simple control: move forward in a circle
            throttle = 0.5f; // Forward
            steering = 0.3f; // Slight turn

            // Send control command to simulator
            fs::messages::ControlCommand cmd(client.get_uuid(), 0.0, steering, throttle);
            client.send_control_command(cmd);

            // Sleep to avoid overwhelming the simulator
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } catch (const std::exception &e) {
        std::cerr << "[Robot] Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
