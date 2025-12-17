#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

std::string generate_uuid() {
    static std::mt19937 gen(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<> dis(0, 15);
    std::uniform_int_distribution<> dis2(8, 11);

    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (int i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4";
    for (int i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (int i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (int i = 0; i < 12; i++) {
        ss << dis(gen);
    }
    return ss.str();
}

std::vector<concord::Point> generate_s_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<concord::Point> path;
    for (int i = 0; i <= 20; ++i) {
        float t = i / 20.0f;
        float x = offset_x + t * 40.0f * scale;
        float y = offset_y + 15.0f * scale * std::sin(t * 2.0f * M_PI);
        path.push_back({x, y});
    }
    return path;
}

int main(int argc, char *argv[]) {
    std::cout << "=== MPPI Controller Stress Test: 30 Robots ===" << std::endl;

    auto rec = std::make_shared<rerun::RecordingStream>("mppi_stress_30", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{800.0f, 800.0f, 300.0f};
    simulator.init(world_datum, world_size);

    const int NUM_ROBOTS = 30;
    const int GRID_SIZE = 6; // 6x5 grid for 30 robots
    const float SPACING = 15.0f;

    std::vector<concord::Point> path_starts;     // Where the path begins
    std::vector<concord::Point> spawn_positions; // Where robots actually spawn (offset from path)
    std::vector<float> spawn_orientations;       // Random orientations for each robot
    std::vector<pigment::RGB> colors;

    // Generate spawn positions in a grid pattern
    std::mt19937 color_gen(42);
    std::uniform_int_distribution<> color_dis(0, 255);
    std::uniform_real_distribution<float> angle_dis(-M_PI, M_PI); // Random orientation [-pi, pi]

    const float SPAWN_OFFSET_X = -10.0f; // Spawn 10m behind path start
    const float SPAWN_OFFSET_Y = -10.0f; // Spawn 10m to the side of path start

    for (int i = 0; i < NUM_ROBOTS; ++i) {
        int row = i / GRID_SIZE;
        int col = i % GRID_SIZE;
        float x = -150.0f + col * SPACING;
        float y = -100.0f + row * SPACING;
        path_starts.push_back({x, y});

        // Spawn position is offset from path start
        spawn_positions.push_back({x + SPAWN_OFFSET_X, y + SPAWN_OFFSET_Y});

        // Generate random orientation for each robot
        spawn_orientations.push_back(angle_dis(color_gen));

        // Generate random colors for variety
        colors.push_back({static_cast<uint8_t>(color_dis(color_gen)), static_cast<uint8_t>(color_dis(color_gen)),
                          static_cast<uint8_t>(color_dis(color_gen))});
    }

    std::cout << "Loading " << NUM_ROBOTS << " tractors..." << std::endl;

    try {
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            auto tractor_info = fs::Loader::load_from_json(
                "examples/machines/tractor.json",
                concord::Pose{spawn_positions[i], concord::Euler{0.0f, 0.0f, spawn_orientations[i]}}, colors[i]);

            tractor_info.uuid = generate_uuid();
            tractor_info.seqid = tractor_info.type + "_" + std::to_string(i);

            simulator.add_robot(tractor_info);

            if ((i + 1) % 10 == 0 || i == NUM_ROBOTS - 1) {
                std::cout << "Loaded " << (i + 1) << "/" << NUM_ROBOTS << " tractors" << std::endl;
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractors: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "All " << NUM_ROBOTS << " tractors loaded successfully\n" << std::endl;

    // All robots follow the same S-shape path from their spawn positions
    std::cout << "Setting up S-shape paths for all robots with MPPI controller..." << std::endl;
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        auto &tractor = simulator.get_robot(i);

        // Set controller type to MPPI
        tractor.tracker->set_controller_type(drivekit::TrackerType::MPPI);

        // Configure MPPI controller parameters (per robot)
        auto mppi_controller = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker->get_controller());
        if (mppi_controller) {
            auto mppi_config = mppi_controller->get_mppi_config();

            // Configuration adapted from examples/test_mppi.cpp, slightly tuned for 30 robots
            mppi_config.horizon_steps = 20;       // Slightly shorter horizon for multi-robot load
            mppi_config.dt = 0.1;                 // Time step (seconds)
            mppi_config.num_samples = 1200;       // Fewer samples to keep computation reasonable
            mppi_config.temperature = 0.15;       // Moderate temperature for stability
            mppi_config.steering_noise = 0.15;    // Reduced noise for less oscillation
            mppi_config.acceleration_noise = 0.1; // Reduced acceleration noise
            mppi_config.ref_velocity = 0.8;       // Reference normalized speed (~80% throttle)

            // Cost weights
            mppi_config.weight_cte = 200.0;         // Cross-track error
            mppi_config.weight_epsi = 180.0;        // Heading error
            mppi_config.weight_vel = 1.0;           // Velocity tracking
            mppi_config.weight_steering = 80.0;     // Steering penalty
            mppi_config.weight_acceleration = 20.0; // Acceleration penalty

            mppi_controller->set_mppi_config(mppi_config);
        } else {
            std::cerr << "Failed to cast controller to MPPI for robot " << i << std::endl;
        }

        // Set S-shape path from path start position (not spawn position)
        auto path = generate_s_shape(path_starts[i].x, path_starts[i].y, 1.0f);
        drivekit::PathGoal path_goal(path, 2.0f, 2.5f, false);
        tractor.tracker->set_path(path_goal);
        tractor.tracker->smoothen(25.0f); // Smooth path for MPPI
    }

    std::cout << "Starting MPPI path following for all " << NUM_ROBOTS << " tractors...\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f;

    int step_count = 0;
    std::vector<float> max_errors(NUM_ROBOTS, 0.0f);
    std::vector<float> total_errors(NUM_ROBOTS, 0.0f);
    std::vector<int> error_samples(NUM_ROBOTS, 0);

    bool all_completed = false;
    while (!all_completed) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        auto tick_start = std::chrono::high_resolution_clock::now();
        simulator.tick(dt);
        simulator.tock(5);
        auto tick_end = std::chrono::high_resolution_clock::now();
        auto tick_duration = std::chrono::duration_cast<std::chrono::microseconds>(tick_end - tick_start).count();

        all_completed = true;
        int completed_count = 0;
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            auto &tractor = simulator.get_robot(i);
            if (!tractor.tracker->is_path_completed()) {
                all_completed = false;

                auto target = tractor.tracker->get_current_target();
                auto pos = tractor.get_position();
                float tracking_error =
                    std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

                max_errors[i] = std::max(max_errors[i], tracking_error);
                total_errors[i] += tracking_error;
                error_samples[i]++;
            } else {
                completed_count++;
            }
        }

        if (step_count % 120 == 0) {
            float avg_tick_time = tick_duration / 1000.0f; // Convert to milliseconds
            std::cout << "\n--- Progress at " << step_count / 60 << "s | Tick: " << avg_tick_time
                      << "ms | Completed: " << completed_count << "/" << NUM_ROBOTS << " ---" << std::endl;

            // Show sample of first 5 robots
            for (int i = 0; i < std::min(5, NUM_ROBOTS); ++i) {
                auto &tractor = simulator.get_robot(i);
                bool completed = tractor.tracker->is_path_completed();
                float avg_error = error_samples[i] > 0 ? total_errors[i] / error_samples[i] : 0.0f;
                std::cout << "  Robot " << i << ": " << (completed ? "✅" : "🚜") << " | Avg Error: " << avg_error
                          << "m" << std::endl;
            }
            if (NUM_ROBOTS > 5) {
                std::cout << "  ... and " << (NUM_ROBOTS - 5) << " more robots" << std::endl;
            }
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== MPPI Controller Stress Test Results (30 Robots) ===" << std::endl;

    float total_max_error = 0.0f;
    float total_avg_error = 0.0f;
    int total_completed = 0;

    for (int i = 0; i < NUM_ROBOTS; ++i) {
        auto &tractor = simulator.get_robot(i);
        bool completed = tractor.tracker->is_path_completed();
        float avg_error = error_samples[i] > 0 ? total_errors[i] / error_samples[i] : 0.0f;

        total_max_error += max_errors[i];
        total_avg_error += avg_error;
        if (completed) total_completed++;
    }

    std::cout << "\nOverall Statistics:" << std::endl;
    std::cout << "  Completed: " << total_completed << "/" << NUM_ROBOTS << " ("
              << (100.0f * total_completed / NUM_ROBOTS) << "%)" << std::endl;
    std::cout << "  Average Max Error: " << (total_max_error / NUM_ROBOTS) << "m" << std::endl;
    std::cout << "  Average Tracking Error: " << (total_avg_error / NUM_ROBOTS) << "m" << std::endl;
    std::cout << "  Total Simulation Steps: " << step_count << std::endl;
    std::cout << "  Total Time: " << (step_count * dt) << "s" << std::endl;

    std::cout << "\n=== Test Complete ===" << std::endl;

    return 0;
}

