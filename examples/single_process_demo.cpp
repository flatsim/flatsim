#include "flatsim/agent.hpp"
#include "flatsim/core/loader.hpp"
#include "flatsim/protocol/types.hpp"
#include "flatsim/server/environment.hpp"

#include <CLI/CLI.hpp>
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {
    CLI::App app{"Flatsim single-process environment + agent demo"};

    std::string config_file;
    float target_x = 10.0f;
    float target_y = 0.0f;

    app.add_option("--config", config_file, "Path to robot configuration JSON")->required()->check(CLI::ExistingFile);
    app.add_option("--target-x", target_x, "Target X position in world coordinates")->default_val(10.0f);
    app.add_option("--target-y", target_y, "Target Y position in world coordinates")->default_val(0.0f);

    CLI11_PARSE(app, argc, argv);

    try {
        // Initialize Rerun logging
        auto rec = std::make_shared<rerun::RecordingStream>("flatsim_single_process_demo", "space");
        rec->spawn().exit_on_failure();
        rec->set_global();

        // World setup (re-use defaults from other examples)
        concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
        concord::Size world_size{100.0f, 100.0f, 300.0f};

        fs::server::EnvironmentServer env(rec);
        env.init(world_datum, world_size);

        // Load robot info from JSON
        std::filesystem::path config_path(config_file);
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "Config file not found: " << config_file << std::endl;
            return 1;
        }

        concord::Pose spawn_pose(0.0f, 0.0f, 0.0f);
        fs::RobotInfo robot_info = fs::Loader::load_from_json(config_path, spawn_pose, std::nullopt);

        // Add robot to environment
        env.add_robot(robot_info);

        if (env.robots().empty() || !env.robots().front()) {
            std::cerr << "Failed to create robot from config" << std::endl;
            return 1;
        }

        const auto robot_id = robot_info.uuid;

        // Configure simple navcon constraints (approximate, not from geometry)
        navcon::RobotConstraints constraints;
        constraints.steering_type = navcon::SteeringType::ACKERMANN;
        constraints.wheelbase = 1.5;
        constraints.track_width = 1.5;
        constraints.max_linear_velocity = 1.0;
        constraints.min_linear_velocity = -1.0;
        constraints.max_linear_acceleration = 1.0;
        constraints.max_angular_velocity = 1.0;
        constraints.max_steering_angle = 30.0 * M_PI / 180.0;
        constraints.max_steering_rate = 1.0;
        constraints.min_turning_radius = robot_info.turning_radius;
        constraints.robot_length = robot_info.bound.size.y;
        constraints.robot_width = robot_info.bound.size.x;

        // Create unified agent with navigation and optional logging
        fs::Agent agent(robot_id, constraints, navcon::TrackerType::CARROT, rec);
        navcon::NavigationGoal goal(concord::Point{target_x, target_y}, 0.5f, 1.0f);
        agent.set_goal(goal);

        std::vector<fs::protocol::RobotCommand> commands;
        std::vector<fs::protocol::RobotState> states;

        auto last_time = std::chrono::steady_clock::now();
        int tick_count = 0;

        std::cout << "[Demo] Starting single-process simulation. Press Ctrl+C to exit.\n";

        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = now - last_time;
            last_time = now;
            float dt_s = dt.count();
            if (dt_s <= 0.0f) {
                dt_s = 0.016f;
            }

            // Step environment with previous commands
            env.step(dt_s, commands, states);

            // Compute new commands from agent(s)
            commands.clear();
            for (const auto &state : states) {
                if (state.id == robot_id) {
                    // Feed state to agent
                    agent.on_state(state, dt_s);

                    commands.push_back(agent.compute_command());
                }
            }

            // Periodic visualization: world + robots
            if (++tick_count % 5 == 0) {
                env.world().tock();
                for (auto &robot : env.robots()) {
                    if (robot) {
                        robot->tock();
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
