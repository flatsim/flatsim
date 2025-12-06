#include "flatsim/core/loader.hpp"
#include "flatsim/ipc/client.hpp"
#include "flatsim/robot/types.hpp"
#include <CLI/CLI.hpp>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <linux/joystick.h>
#include <thread>
#include <unistd.h>

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
    bool use_joystick = false;
    std::string tcp_host;

    app.add_option("--config", config_file, "Path to configuration file")->required()->check(CLI::ExistingFile);

    app.add_option("--pose", position_str, "Initial pose in format: x,y,r")->required();
    app.add_option("--color", color_str, "Robot color in format: r,g,b");
    app.add_option("--uuid", custom_uuid, "Custom UUID for the robot");
    app.add_flag("--joystick", use_joystick, "Enable joystick control");
    app.add_option("--tcp", tcp_host, "Use TCP transport with specified host (e.g., 127.0.0.1 or 192.168.1.10)");

    CLI11_PARSE(app, argc, argv);

    // Joystick initialization
    int js_fd = -1;
    unsigned char num_axes = 0, num_buttons = 0;

    if (use_joystick) {
        const char *js_device = "/dev/input/js0";
        js_fd = open(js_device, O_RDONLY | O_NONBLOCK);
        if (js_fd < 0) {
            std::cerr << "Failed to open joystick device " << js_device << std::endl;
            return 1;
        }

        // Query number of axes/buttons
        ioctl(js_fd, JSIOCGAXES, &num_axes);
        ioctl(js_fd, JSIOCGBUTTONS, &num_buttons);
    }

    try {
        // Parse CLI arguments
        concord::Point position = parse_position(position_str);
        pigment::RGB color = parse_color(color_str);

        // Load RobotInfo from JSON file
        std::filesystem::path config_path(config_file);
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "Config file not found: " << config_file << std::endl;
            return 1;
        }

        // Load robot info with overrides
        concord::Pose spawn_pose(position.x, position.y, position.z);
        // concord::Heading spawn_
        fs::RobotInfo robot_info =
            fs::Loader::load_from_json(config_path, spawn_pose, color); // Empty name, will use seqid

        // Override UUID if provided
        if (!custom_uuid.empty()) {
            robot_info.uuid = custom_uuid;
        }

        // Create ZMQ client
        fs::Client client;
        bool use_tcp = !tcp_host.empty();
        if (!client.init(use_tcp, tcp_host.empty() ? "127.0.0.1" : tcp_host)) {
            std::cerr << "Failed to initialize client" << std::endl;
            return 1;
        }

        // Spawn robot in simulator
        if (!client.spawn_robot(robot_info)) {
            std::cerr << "Failed to spawn robot" << std::endl;
            return 1;
        }

        // Control loop
        auto last_time = std::chrono::steady_clock::now();
        float steering = 0.0f;
        float throttle = 0.0f;

        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = now - last_time;
            last_time = now;

            // Send heartbeat to simulator
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
            double current_time = elapsed.count() / 1000.0;
            client.send_heartbeat(current_time);

            // Receive physics state from simulator
            auto physics_state = client.receive_physics_state();

            // Read joystick input if enabled
            if (use_joystick) {
                js_event e;
                ssize_t bytes = read(js_fd, &e, sizeof(e));
                if (bytes == sizeof(e)) {
                    auto type = e.type & ~JS_EVENT_INIT;

                    if (type == JS_EVENT_AXIS && e.number < num_axes) {
                        int axis = int(e.number);
                        float value = e.value / 32767.0f;

                        if (axis == 0) {
                            // Axis 0 = Steering
                            steering = value;
                        } else if (axis == 1) {
                            // Axis 1 = Throttle (NOT inverted, use raw value like mvs.cpp)
                            throttle = -value;
                            // Apply deadzone
                            throttle = (std::fabs(throttle) < 0.05f) ? 0.0f : throttle;
                        }
                    } else if (type == JS_EVENT_BUTTON && e.number < num_buttons) {
                        int button = int(e.number);
                        bool pressed = e.value != 0;

                        if (pressed) {
                            // Button actions can be added here
                            // Example: Button 0 = reset to zero
                            if (button == 0) {
                                steering = 0.0f;
                                throttle = 0.0f;
                            }
                        }
                    }
                }
            }

            // Send control command to simulator
            fs::messages::ControlCommand cmd(client.get_uuid(), 0.0, steering, throttle);
            client.send_control_command(cmd);

            // Sleep to avoid overwhelming the simulator
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;

        // Cleanup joystick
        if (js_fd >= 0) {
            close(js_fd);
        }
        return 1;
    }

    // Close joystick
    if (js_fd >= 0) {
        close(js_fd);
    }

    return 0;
}
