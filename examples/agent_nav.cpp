#include "flatsim/core/loader.hpp"
#include "flatsim/ipc/adapters.hpp"
#include "flatsim/ipc/client.hpp"
#include "flatsim/agent/nav_agent.hpp"
#include "flatsim/agent/logging_agent.hpp"

#include <CLI/CLI.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {
    CLI::App app{"Navcon-based ZMQ robot agent (navigation + logging on client)"};

    std::string config_file;
    std::string position_str;
    std::string tcp_host;
    double target_x = 10.0;
    double target_y = 0.0;
    bool enable_logging = false;

    app.add_option("--config", config_file, "Path to robot configuration JSON")
        ->required()
        ->check(CLI::ExistingFile);
    app.add_option("--pose", position_str, "Initial pose in format: x,y,r")->required();
    app.add_option("--tcp", tcp_host,
                   "Use TCP transport with specified host (e.g., 127.0.0.1 or 192.168.1.10)");
    app.add_option("--target-x", target_x, "Target X position in world coordinates")->default_val(10.0);
    app.add_option("--target-y", target_y, "Target Y position in world coordinates")->default_val(0.0);
    app.add_flag("--log-agent", enable_logging, "Enable client-side Rerun logging for this agent");

    CLI11_PARSE(app, argc, argv);

    try {
        // Parse pose "x,y,r"
        std::istringstream iss(position_str);
        std::string token;
        std::vector<double> values;
        while (std::getline(iss, token, ',')) {
            values.push_back(std::stod(token));
        }
        if (values.size() != 3) throw std::runtime_error("Pose must be in format: x,y,r");
        concord::Pose spawn_pose(values[0], values[1], values[2]);

        // Load robot info from JSON
        std::filesystem::path config_path(config_file);
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "Config file not found: " << config_file << std::endl;
            return 1;
        }

        fs::RobotInfo robot_info = fs::Loader::load_from_json(config_path, spawn_pose, std::nullopt);

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

        const auto robot_id = client.get_uuid();

        // Configure basic navcon constraints from RobotInfo (approximate)
        navcon::RobotConstraints constraints;
        constraints.steering_type = navcon::SteeringType::ACKERMANN;

        if (!robot_info.wheels.empty()) {
            double max_y = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_x = std::numeric_limits<double>::infinity();

            for (const auto &w : robot_info.wheels) {
                max_y = std::max(max_y, static_cast<double>(w.pose.point.y));
                min_y = std::min(min_y, static_cast<double>(w.pose.point.y));
                max_x = std::max(max_x, static_cast<double>(w.pose.point.x));
                min_x = std::min(min_x, static_cast<double>(w.pose.point.x));
            }

            double wheelbase = std::abs(max_y - min_y);
            double track_width = std::abs(max_x - min_x);

            constraints.wheelbase = wheelbase > 0.1 ? wheelbase : 1.5;
            constraints.track_width = track_width > 0.1 ? track_width : 1.5;
        } else {
            constraints.wheelbase = 1.5;
            constraints.track_width = 1.5;
        }

        constraints.max_linear_velocity = 1.0;
        constraints.min_linear_velocity = -1.0;
        constraints.max_linear_acceleration = 1.0;
        constraints.max_angular_velocity = 1.0;

        double max_steer = 0.0;
        for (float a : robot_info.controls.steerings_max) {
            max_steer = std::max(max_steer, static_cast<double>(std::abs(a)));
        }
        if (max_steer <= 0.0) {
            max_steer = 30.0 * M_PI / 180.0;
        }
        constraints.max_steering_angle = max_steer;
        constraints.max_steering_rate = 1.0;

        constraints.min_turning_radius = robot_info.turning_radius;
        constraints.robot_length = robot_info.bound.size.y;
        constraints.robot_width = robot_info.bound.size.x;

        // Create navigation agent
        fs::agent::NavAgent nav_agent(robot_id, constraints, navcon::TrackerType::CARROT);
        navcon::NavigationGoal goal(concord::Point{target_x, target_y}, 0.5f, 1.0f);
        nav_agent.set_goal(goal);

        // Optional client-side Rerun logging
        std::shared_ptr<rerun::RecordingStream> rec;
        std::unique_ptr<fs::agent::LoggingAgent> log_agent;
        if (enable_logging) {
            rec = std::make_shared<rerun::RecordingStream>("flatsim_agent_nav", robot_id);
            rec->spawn().exit_on_failure();
            rec->set_global();
            log_agent = std::make_unique<fs::agent::LoggingAgent>(robot_id, rec);
        }

        auto last_time = std::chrono::steady_clock::now();

        std::cout << "[Agent] Started for robot " << robot_id << ". Press Ctrl+C to exit.\n";

        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = now - last_time;
            last_time = now;
            float dt_s = dt.count();
            if (dt_s <= 0.0f) dt_s = 0.016f;

            // Heartbeat
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
            double current_time = elapsed.count() / 1000.0;
            client.send_heartbeat(current_time);

            // Receive latest physics state (non-blocking)
            auto physics_state_opt = client.receive_physics_state();
            if (!physics_state_opt.has_value()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // Convert to protocol state
            fs::protocol::RobotState proto_state = fs::ipc::to_protocol(*physics_state_opt);

            // Update agents
            nav_agent.on_state(proto_state, dt_s);
            if (log_agent) {
                log_agent->on_state(proto_state, dt_s);
            }

            // Compute and send command
            fs::protocol::RobotCommand proto_cmd = nav_agent.compute_command();
            fs::messages::ControlCommand cmd = fs::ipc::from_protocol(proto_cmd);
            client.send_control_command(cmd);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

