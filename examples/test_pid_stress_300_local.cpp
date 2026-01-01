// PID Controller Stress Test: 300 Robots (LOCAL mode)
//
// Migrated from `examples_old/test_pid_stress_300.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_pid_stress_300_local

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
#include <random>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"
#include <vector>
#include "flatsim/utils.hpp"

static std::vector<datapod::Point> generate_s_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<datapod::Point> path;
    path.reserve(21);
    for (int i = 0; i <= 20; ++i) {
        const float t = i / 20.0f;
        const float x = offset_x + t * 40.0f * scale;
        const float y = offset_y + 15.0f * scale * std::sin(t * 2.0f * static_cast<float>(std::numbers::pi));
        path.push_back({x, y});
    }
    return path;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    std::cout << "=== PID Controller Stress Test: 300 Robots (LOCAL mode) ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(1500.0f, 1500.0f, datum);

    constexpr int kNumRobots = 300;
    constexpr int kGridSize = 20; // 20x15
    constexpr float kSpacing = 12.0f;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> color_dis(0, 255);

    std::vector<agent::Agent *> tractors;
    tractors.reserve(kNumRobots);

    auto load_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kNumRobots; ++i) {
        const int row = i / kGridSize;
        const int col = i % kGridSize;
        const float x = -500.0f + col * kSpacing;
        const float y = -400.0f + row * kSpacing;
        const pigment::RGB color{static_cast<uint8_t>(color_dis(rng)), static_cast<uint8_t>(color_dis(rng)),
                                 static_cast<uint8_t>(color_dis(rng))};
        const std::string uuid = "tractor_" + std::to_string(i);
        auto &tractor = sim.spawn_agent("examples/machines/tractor.json", utils::make_pose_2d(x, y, 0.0), uuid, color);
        tractors.push_back(&tractor);
        if ((i + 1) % 50 == 0 || i == kNumRobots - 1) {
            std::cout << "[Setup] Spawned " << (i + 1) << "/" << kNumRobots << std::endl;
        }
    }

    auto load_end = std::chrono::high_resolution_clock::now();
    const auto load_ms = std::chrono::duration_cast<std::chrono::milliseconds>(load_end - load_start).count();
    std::cout << "[Setup] Spawned all robots in " << load_ms << "ms" << std::endl;

    for (int i = 0; i < kNumRobots; ++i) {
        auto &tractor = *tractors[i];
        auto params = tractor.tracker()->get_controller_params();
        params.linear_kp = 2.5f;
        params.angular_kp = 1.8f;
        params.angular_kd = 0.2f;
        tractor.tracker()->set_controller_params(params);

        tractor.controls().tracker().set_controller_type(drivekit::TrackerType::PID);
        tractor.controls().tracker().set_enabled(true);
        tractor.set_navigation_enabled(true);

        const auto pos = tractor.get_position();
        const auto path =
            generate_s_shape(static_cast<float>(pos.point.x), static_cast<float>(pos.point.y), 1.0f);
        tractor.tracker()->set_path(drivekit::PathGoal(path, 2.0f, 2.5f, false));

        if ((i + 1) % 100 == 0) {
            std::cout << "[Setup] Configured " << (i + 1) << "/" << kNumRobots << " paths" << std::endl;
        }
    }

    const float dt = 0.016f;
    auto start_time = std::chrono::steady_clock::now();
    int step_count = 0;
    std::vector<float> max_errors(kNumRobots, 0.0f);
    std::vector<float> total_errors(kNumRobots, 0.0f);
    std::vector<int> error_samples(kNumRobots, 0);

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 600) {
            std::cout << "[Run] Timeout reached" << std::endl;
            break;
        }

        const auto tick_start = std::chrono::high_resolution_clock::now();
        sim.tick(dt);
        if (step_count % 20 == 0) {
            sim.tock();
        }
        const auto tick_end = std::chrono::high_resolution_clock::now();
        const auto tick_us =
            std::chrono::duration_cast<std::chrono::microseconds>(tick_end - tick_start).count();

        bool all_completed = true;
        int completed_count = 0;
        for (int i = 0; i < kNumRobots; ++i) {
            auto &tractor = *tractors[i];
            if (!tractor.tracker()->is_path_completed()) {
                all_completed = false;
                const auto target = tractor.tracker()->get_current_target();
                const auto pos = tractor.get_position();
                const float err = std::sqrt(std::pow(static_cast<float>(target.x - pos.point.x), 2.0f) +
                                            std::pow(static_cast<float>(target.y - pos.point.y), 2.0f));
                max_errors[i] = std::max(max_errors[i], err);
                total_errors[i] += err;
                error_samples[i]++;
            } else {
                completed_count++;
            }
        }

        if (step_count % 120 == 0) {
            const float tick_ms = static_cast<float>(tick_us) / 1000.0f;
            const float fps = (tick_ms > 0.0f) ? (1000.0f / tick_ms) : 0.0f;
            std::cout << "\n[Run] " << (step_count / 60) << "s tick=" << tick_ms << "ms (" << fps
                      << " fps) completed=" << completed_count << "/" << kNumRobots << std::endl;
        }

        if (all_completed) {
            std::cout << "[Run] All robots completed their paths" << std::endl;
            break;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    float total_max_error = 0.0f;
    float total_avg_error = 0.0f;
    int total_completed = 0;
    for (int i = 0; i < kNumRobots; ++i) {
        const bool completed = tractors[i]->tracker()->is_path_completed();
        const float avg_error = (error_samples[i] > 0) ? (total_errors[i] / error_samples[i]) : 0.0f;
        total_max_error += max_errors[i];
        total_avg_error += avg_error;
        if (completed) {
            total_completed++;
        }
    }

    std::cout << "\n=== PID Stress Results (300 Robots) ===" << std::endl;
    std::cout << "Completed: " << total_completed << "/" << kNumRobots << " (" << (100.0f * total_completed / kNumRobots)
              << "%)" << std::endl;
    std::cout << "Average max error: " << (total_max_error / kNumRobots) << "m" << std::endl;
    std::cout << "Average tracking error: " << (total_avg_error / kNumRobots) << "m" << std::endl;

    return 0;
}

