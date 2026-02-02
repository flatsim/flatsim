// PID Controller Path Following Test
//
// Run:
//   ./build/test_pid

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <cmath>
#include <drivekit.hpp>
#include <iostream>
#include <thread>
#include <vector>

static std::vector<datapod::Point> generate_s_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    for (int i = 0; i <= 20; ++i) {
        const float t = i / 20.0f;
        const float x = offset_x + t * 40.0f * scale;
        const float y = offset_y + 15.0f * scale * std::sin(t * 2.0f * static_cast<float>(M_PI));
        path.push_back({x, y});
    }
    return path;
}

static std::vector<datapod::Point> generate_u_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    for (int i = 0; i <= 20; ++i) {
        const float t = i / 20.0f;
        const float angle = static_cast<float>(M_PI) * t;
        const float x = offset_x + 15.0f * scale * std::sin(angle);
        const float y = offset_y - 15.0f * scale * std::cos(angle) + 15.0f * scale;
        path.push_back({x, y});
    }
    return path;
}

static std::vector<datapod::Point> generate_o_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    for (int i = 0; i <= 24; ++i) {
        const float t = i / 24.0f;
        const float angle = 2.0f * static_cast<float>(M_PI) * t;
        const float x = offset_x + 15.0f * scale * std::cos(angle);
        const float y = offset_y + 15.0f * scale * std::sin(angle);
        path.push_back({x, y});
    }
    return path;
}

static std::vector<datapod::Point> generate_l_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    for (int i = 0; i <= 10; ++i) {
        const float y = offset_y + i * 3.0f * scale;
        path.push_back({offset_x, y});
    }
    for (int i = 1; i <= 10; ++i) {
        const float x = offset_x + i * 3.0f * scale;
        path.push_back({x, offset_y + 30.0f * scale});
    }
    return path;
}

int main() {
    std::cout << "=== PID Controller Path Following Test ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    const std::vector<datapod::Point> spawn_positions = {
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {-50.0f, 50.0f},
        {50.0f, 50.0f},
    };
    const std::vector<pigment::RGB> colors = {
        {255, 0, 0},
        {0, 255, 0},
        {0, 0, 255},
        {255, 255, 0},
    };

    std::vector<agent::Agent *> tractors;
    tractors.reserve(4);

    for (int i = 0; i < 4; ++i) {
        const auto uuid = std::string("pid_") + std::to_string(i);
        const datapod::Pose spawn_pose = utils::make_pose_2d(spawn_positions[i].x, spawn_positions[i].y, 0.0f);
        auto &tractor = sim.spawn_agent("machines/urdf/tractor.urdf", spawn_pose, uuid, colors[i]);
        tractors.push_back(&tractor);
        std::cout << "[Spawn] Tractor " << i << " uuid=" << tractor.uuid() << " at (" << spawn_positions[i].x << ","
                  << spawn_positions[i].y << ")\n";
    }

    const std::vector<std::vector<datapod::Point>> paths = {
        generate_s_shape(spawn_positions[0].x, spawn_positions[0].y, 1.0f),
        generate_u_shape(spawn_positions[1].x, spawn_positions[1].y, 1.0f),
        generate_o_shape(spawn_positions[2].x, spawn_positions[2].y, 1.0f),
        generate_l_shape(spawn_positions[3].x, spawn_positions[3].y, 1.0f),
    };
    const std::vector<std::string> shape_names = {"S-shape", "U-shape", "O-shape", "L-shape"};

    for (int i = 0; i < 4; ++i) {
        auto &tractor = *tractors[i];

        // Configure PID controller
        tractor.tracker()->set_controller_type(drivekit::TrackerType::PID);
        tractor.set_tracker_enabled(true);

        auto params = tractor.tracker()->get_controller_params();
        params.linear_kp = 2.5f;
        params.angular_kp = 1.8f;
        params.angular_kd = 0.2f;
        tractor.tracker()->set_controller_params(params);

        drivekit::PathGoal path(paths[i], 2.0f, 2.5f, false);
        tractor.tracker()->set_path(path);

        std::cout << "[Path] Tractor " << i << " following " << shape_names[i] << " (" << paths[i].size()
                  << " waypoints)\n";
    }

    std::cout << "\n[Run] Starting PID path following...\n" << std::endl;

    const float dt = 0.016f;
    int step_count = 0;

    std::vector<float> max_errors(4, 0.0f);
    std::vector<float> total_errors(4, 0.0f);
    std::vector<int> error_samples(4, 0);

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 180) {
            std::cout << "[Run] Timeout reached\n";
            break;
        }

        sim.tick(dt);
        sim.tock();

        bool all_completed = true;
        for (int i = 0; i < 4; ++i) {
            auto &tractor = *tractors[i];
            if (!tractor.tracker()->is_path_completed()) {
                all_completed = false;
                const auto target = tractor.tracker()->get_current_target();
                const auto pos = tractor.get_position();
                const float dx = target.x - static_cast<float>(pos.point.x);
                const float dy = target.y - static_cast<float>(pos.point.y);
                const float tracking_error = std::sqrt(dx * dx + dy * dy);
                max_errors[i] = std::max(max_errors[i], tracking_error);
                total_errors[i] += tracking_error;
                error_samples[i]++;
            }
        }

        if (all_completed) {
            break;
        }

        if (step_count % 120 == 0) {
            std::cout << "\n--- Progress at " << (step_count / 60) << "s ---" << std::endl;
            for (int i = 0; i < 4; ++i) {
                auto &tractor = *tractors[i];
                const auto pos = tractor.get_position();
                const bool completed = tractor.tracker()->is_path_completed();
                const float avg_error = error_samples[i] > 0 ? total_errors[i] / error_samples[i] : 0.0f;
                std::cout << "Tractor " << i << " (" << shape_names[i] << "): " << (completed ? "COMPLETED" : "RUNNING")
                          << " | Pos(" << pos.point.x << "," << pos.point.y << ")"
                          << " | Avg Error: " << avg_error << "m\n";
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== PID Controller Test Results ===" << std::endl;
    for (int i = 0; i < 4; ++i) {
        auto &tractor = *tractors[i];
        const bool completed = tractor.tracker()->is_path_completed();
        const float avg_error = error_samples[i] > 0 ? total_errors[i] / error_samples[i] : 0.0f;
        const auto final_pos = tractor.get_position();

        std::cout << "\nTractor " << i << " (" << shape_names[i] << "):" << std::endl;
        std::cout << "  Status: " << (completed ? "Completed" : "Not completed") << std::endl;
        std::cout << "  Max error: " << max_errors[i] << "m" << std::endl;
        std::cout << "  Avg error: " << avg_error << "m" << std::endl;
        std::cout << "  Final pos: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;
    }

    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
