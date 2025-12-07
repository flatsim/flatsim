#include "flatsim/core/loader.hpp"
#include "flatsim/server/environment.hpp"
#include "flatsim/server/simulation.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>

int main(int argc, char *argv[]) {
    CLI::App app{"Flatsim environment-only server using EnvironmentServer"};

    std::vector<std::string> machine_configs;
    float world_width = 100.0f;
    float world_height = 100.0f;

    app.add_option("--config", machine_configs, "Robot JSON configuration files to load")
        ->required()
        ->check(CLI::ExistingFile);
    app.add_option("--world-width", world_width, "World width in meters")->default_val(100.0f);
    app.add_option("--world-height", world_height, "World height in meters")->default_val(100.0f);

    CLI11_PARSE(app, argc, argv);

    try {
        // Initialize Rerun logging
        auto rec = std::make_shared<rerun::RecordingStream>("flatsim_server_env", "space");
        rec->spawn().exit_on_failure();
        rec->set_global();

        // World setup (same datum as other examples)
        concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
        concord::Size world_size{world_width, world_height, 300.0f};

        fs::server::EnvironmentServer env(rec);
        env.init(world_datum, world_size);

        // Load and add robots
        for (const auto &path_str : machine_configs) {
            std::filesystem::path config_path(path_str);
            if (!std::filesystem::exists(config_path)) {
                std::cerr << "Config file not found: " << config_path << std::endl;
                continue;
            }

            // Spawn robots at origin by default; higher-level logic can teleport them later.
            concord::Pose spawn_pose(0.0f, 0.0f, 0.0f);
            fs::RobotInfo info = fs::Loader::load_from_json(config_path, spawn_pose, std::nullopt);
            env.add_robot(info);

            std::cout << "[server_env] Loaded robot from " << config_path << " with uuid=" << info.uuid << "\n";
        }

        std::cout << "[server_env] Environment initialized with " << env.robots().size() << " robots.\n";
        std::cout << "[server_env] Running headless physics loop. Press Ctrl+C to exit.\n";

        // Simple loop: no external commands, pure physics + visualization.
        fs::server::run(
            env,
            [](float /*dt*/, const std::vector<fs::protocol::RobotState> & /*states*/) {
                // No external control in this example; continue forever.
                return true;
            },
            30);

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

