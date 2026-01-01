// PID Controller Differential Drive Test (LOCAL mode)
//
// Migrated from `examples_old/test_pid_diff.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_pid_diff_local

#include "flatsim/agent.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <algorithm>
#include "flatsim/utils.hpp"
#include <chrono>
#include "flatsim/utils.hpp"
#include <cmath>
#include "flatsim/utils.hpp"
#include <cstdint>
#include "flatsim/utils.hpp"
#include <iostream>
#include "flatsim/utils.hpp"
#include <numbers>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"
#include <vector>
#include "flatsim/utils.hpp"

static std::vector<datapod::Point> generate_s_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    for (int i = 0; i <= 20; ++i) {
        const float t = i / 20.0f;
        const float x = offset_x + t * 40.0f * scale;
        const float y = offset_y + 15.0f * scale * std::sin(t * 2.0f * static_cast<float>(std::numbers::pi));
        path.push_back({x, y});
    }
    return path;
}

static std::vector<datapod::Point> generate_u_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    for (int i = 0; i <= 20; ++i) {
        const float t = i / 20.0f;
        const float angle = static_cast<float>(std::numbers::pi) * t;
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
        const float angle = 2.0f * static_cast<float>(std::numbers::pi) * t;
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

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    std::cout << "=== PID Differential Drive Test (LOCAL mode) ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    const std::vector<datapod::Point> spawns = {{-50.0f, -50.0f}, {50.0f, -50.0f}, {-50.0f, 50.0f}, {50.0f, 50.0f}};
    const std::vector<pigment::RGB> colors = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}};
    const std::vector<const char *> shape_names = {"S-shape", "U-shape", "O-shape", "L-shape"};

    std::vector<std::vector<datapod::Point>> paths = {
        generate_s_shape(static_cast<float>(spawns[0].x), static_cast<float>(spawns[0].y), 1.0f),
        generate_u_shape(static_cast<float>(spawns[1].x), static_cast<float>(spawns[1].y), 1.0f),
        generate_o_shape(static_cast<float>(spawns[2].x), static_cast<float>(spawns[2].y), 1.0f),
        generate_l_shape(static_cast<float>(spawns[3].x), static_cast<float>(spawns[3].y), 1.0f)};

    std::vector<agent::Agent *> huskies;
    huskies.reserve(4);
    for (int i = 0; i < 4; ++i) {
        const std::string uuid = "husky_" + std::to_string(i);
        auto &husky = sim.spawn_agent("examples/machines/husky.json", utils::make_pose_2d(spawns[i].x, spawns[i].y, 0.0), uuid,
                                      colors[i]);
        huskies.push_back(&husky);
    }

    for (int i = 0; i < 4; ++i) {
        auto &husky = *huskies[i];
        auto params = husky.tracker()->get_controller_params();
        params.linear_kp = 2.5f;
        params.angular_kp = 1.8f;
        params.angular_kd = 0.2f;
        husky.tracker()->set_controller_params(params);

        husky.controls().tracker().set_controller_type(drivekit::TrackerType::PID);
        husky.controls().tracker().set_enabled(true);
        husky.set_navigation_enabled(true);

        husky.tracker()->set_path(drivekit::PathGoal(paths[i], 2.0f, 2.5f, false));
        std::cout << "[Setup] Husky " << i << " following " << shape_names[i] << " (" << paths[i].size()
                  << " waypoints)" << std::endl;
    }

    const float dt = 0.016f;
    auto start_time = std::chrono::steady_clock::now();
    int step_count = 0;
    std::vector<float> max_errors(4, 0.0f);
    std::vector<float> total_errors(4, 0.0f);
    std::vector<int> error_samples(4, 0);

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 180) {
            std::cout << "[Run] Timeout reached" << std::endl;
            break;
        }

        sim.tick(dt);
        if (step_count % 5 == 0) {
            sim.tock();
        }

        bool all_completed = true;
        for (int i = 0; i < 4; ++i) {
            auto &husky = *huskies[i];
            if (!husky.tracker()->is_path_completed()) {
                all_completed = false;
                const auto target = husky.tracker()->get_current_target();
                const auto pos = husky.get_position();
                const float err = std::sqrt(std::pow(static_cast<float>(target.x - pos.point.x), 2.0f) +
                                            std::pow(static_cast<float>(target.y - pos.point.y), 2.0f));
                max_errors[i] = std::max(max_errors[i], err);
                total_errors[i] += err;
                error_samples[i]++;
            }
        }

        if (step_count % 120 == 0) {
            std::cout << "\n[Run] " << (step_count / 60) << "s" << std::endl;
            for (int i = 0; i < 4; ++i) {
                const auto pos = huskies[i]->get_position();
                const bool completed = huskies[i]->tracker()->is_path_completed();
                const float avg_error = (error_samples[i] > 0) ? (total_errors[i] / error_samples[i]) : 0.0f;
                std::cout << "  Husky " << i << " (" << shape_names[i] << "): " << (completed ? "completed" : "running")
                          << " pos=(" << pos.point.x << "," << pos.point.y << ") avg_error=" << avg_error << "m"
                          << std::endl;
            }
        }

        if (all_completed) {
            std::cout << "[Run] All huskies completed their paths" << std::endl;
            break;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== PID Diff Results ===" << std::endl;
    for (int i = 0; i < 4; ++i) {
        const bool completed = huskies[i]->tracker()->is_path_completed();
        const float avg_error = (error_samples[i] > 0) ? (total_errors[i] / error_samples[i]) : 0.0f;
        std::cout << "Husky " << i << " (" << shape_names[i] << "): " << (completed ? "completed" : "not completed")
                  << " max_error=" << max_errors[i] << "m avg_error=" << avg_error << "m" << std::endl;
    }

    return 0;
}

