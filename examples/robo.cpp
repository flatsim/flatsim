#include "flatsim/loader.hpp"
#include "flatsim/robot/systems/client.hpp"
#include "flatsim/types.hpp"
#include <CLI/CLI.hpp>
#include <chrono>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <thread>

// Parse position argument "x,y,r"
concord::Point parse_position(const std::string &pos_str) {
    std::istringstream iss(pos_str);
    std::string token;
    std::vector<double> values;
    while (std::getline(iss, token, ',')) {
        values.push_back(std::stod(token));
    }
    if (values.size() != 3) throw std::runtime_error("Position must be in format: x,y,z");
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
    if (values.size() != 3) throw std::runtime_error("Color must be in format: r,g,b");
    return pigment::RGB{static_cast<uint8_t>(values[0]), static_cast<uint8_t>(values[1]),
                        static_cast<uint8_t>(values[2])};
}

int main(int argc, char *argv[]) {
    CLI::App app{"Robot spawner and controller"};

    std::string config_file;
    std::string position_str;
    std::string color_str;
    std::string custom_uuid;

    app.add_option("--config", config_file, "Path to configuration file")->required()->check(CLI::ExistingFile);

    app.add_option("--pose", position_str, "Initial pose in format: x,y,r")->required();
    app.add_option("--color", color_str, "Robot color in format: r,g,b");
    app.add_option("--uuid", custom_uuid, "Custom UUID for the robot");

    CLI11_PARSE(app, argc, argv);

    spdlog::info("Starting robot process...");
    spdlog::info("Config file: " + config_file);
    spdlog::info("Position: " + position_str);
    if (!color_str.empty()) {
        spdlog::info("Color: " + color_str);
    }

    try {
        // Parse CLI arguments
        concord::Point position = parse_position(position_str);
        pigment::RGB color = parse_color(color_str);

        // Load RobotInfo from JSON file
        std::filesystem::path config_path(config_file);
        if (!std::filesystem::exists(config_path)) {
            spdlog::error("Config file not found: " + config_file);
            return 1;
        }

        // Load robot info with overrides
        concord::Pose spawn_pose(position.x, position.y, position.z);
        // concord::Heading spawn_
        fs::RobotInfo robot_info =
            fs::Loader::load_from_json(config_path, spawn_pose, "", color); // Empty name, will use seqid

        // Override UUID if provided
        if (!custom_uuid.empty()) {
            robot_info.uuid = custom_uuid;
        }

        spdlog::info("Loaded robot: " + robot_info.type + " (UUID: " + robot_info.uuid + ")");

        // Create ZMQ client
        fs::Client client;
        if (!client.init()) {
            spdlog::error("Failed to initialize client");
            return 1;
        }

        // Spawn robot in simulator
        spdlog::info("Spawning robot in simulator...");
        if (!client.spawn_robot(robot_info)) {
            spdlog::error("Failed to spawn robot");
            return 1;
        }

        spdlog::info("Robot spawned successfully!");
        spdlog::info("Running control loop (press Ctrl+C to exit)...");

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
                // spdlog::debug("Position: {:.2f}, {:.2f}", physics_state->pose.point.x,
                //               physics_state->pose.point.y);
            }

            // Default control: stationary (no movement)
            throttle = 0.0f; // No forward/backward movement
            steering = 0.0f; // No turning

            // Send control command to simulator
            fs::messages::ControlCommand cmd(client.get_uuid(), 0.0, steering, throttle);
            client.send_control_command(cmd);

            // Sleep to avoid overwhelming the simulator
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } catch (const std::exception &e) {
        spdlog::error("Error: " + std::string(e.what()));
        return 1;
    }

    return 0;
}
