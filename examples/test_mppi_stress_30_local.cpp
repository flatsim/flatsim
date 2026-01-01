// MPPI Controller Stress Test: 30 Robots (LOCAL mode)
//
// Migrated from `examples_old/test_mppi_stress_30.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_mppi_stress_30_local

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <random>
#include <thread>
#include <vector>

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
    std::cout << "=== MPPI Controller Stress Test: 30 Robots (LOCAL mode) ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(800.0f, 800.0f, datum);

    constexpr int kNumRobots = 30;
    constexpr int kGridSize = 6; // 6x5
    constexpr float kSpacing = 15.0f;
    constexpr float kSpawnOffsetX = -10.0f;
    constexpr float kSpawnOffsetY = -10.0f;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> color_dis(0, 255);
    std::uniform_real_distribution<float> angle_dis(-static_cast<float>(std::numbers::pi),
                                                    static_cast<float>(std::numbers::pi));

    std::vector<agent::Agent *> tractors;
    tractors.reserve(kNumRobots);
    std::vector<datapod::Point> path_starts;
    path_starts.reserve(kNumRobots);

    for (int i = 0; i < kNumRobots; ++i) {
        const int row = i / kGridSize;
        const int col = i % kGridSize;
        const float x = -150.0f + col * kSpacing;
        const float y = -100.0f + row * kSpacing;
        path_starts.push_back({x, y});

        const float sx = x + kSpawnOffsetX;
        const float sy = y + kSpawnOffsetY;
        const float yaw = angle_dis(rng);
        const pigment::RGB color{static_cast<uint8_t>(color_dis(rng)), static_cast<uint8_t>(color_dis(rng)),
                                 static_cast<uint8_t>(color_dis(rng))};

        const std::string uuid = "tractor_" + std::to_string(i);
        auto &tractor =
            sim.spawn_agent("examples/machines/tractor.json", utils::make_pose_2d(sx, sy, yaw), uuid, color);
        tractors.push_back(&tractor);
    }

    std::cout << "[Setup] Spawned " << tractors.size() << " tractors" << std::endl;

    for (int i = 0; i < kNumRobots; ++i) {
        auto &tractor = *tractors[i];
        tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MPPI);
        tractor.controls().tracker().set_enabled(true);
        tractor.set_navigation_enabled(true);

        auto *mppi = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker()->get_controller());
        if (!mppi) {
            std::cerr << "[Warn] Failed to cast controller to MPPI for robot " << i << std::endl;
            continue;
        }

        auto cfg = mppi->get_mppi_config();
        cfg.horizon_steps = 20;
        cfg.dt = 0.1;
        cfg.num_samples = 1200;
        cfg.temperature = 0.15;
        cfg.steering_noise = 0.15;
        cfg.acceleration_noise = 0.1;
        cfg.ref_velocity = 0.8;
        cfg.weight_cte = 200.0;
        cfg.weight_epsi = 180.0;
        cfg.weight_vel = 1.0;
        cfg.weight_steering = 80.0;
        cfg.weight_acceleration = 20.0;
        mppi->set_mppi_config(cfg);

        const auto path = generate_s_shape(static_cast<float>(path_starts[i].x), static_cast<float>(path_starts[i].y));
        tractor.tracker()->set_path(drivekit::PathGoal(path, 2.0f, 2.5f, false));
        tractor.tracker()->smoothen(25.0f);
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
        if (elapsed_s > 300) {
            std::cout << "[Run] Timeout reached" << std::endl;
            break;
        }

        const auto tick_start = std::chrono::high_resolution_clock::now();
        sim.tick(dt);
        if (step_count % 5 == 0) {
            sim.tock();
        }
        const auto tick_end = std::chrono::high_resolution_clock::now();
        const auto tick_us = std::chrono::duration_cast<std::chrono::microseconds>(tick_end - tick_start).count();

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
            std::cout << "\n[Run] " << (step_count / 60) << "s"
                      << " tick=" << (tick_us / 1000.0f) << "ms"
                      << " completed=" << completed_count << "/" << kNumRobots << std::endl;

            for (int i = 0; i < std::min(5, kNumRobots); ++i) {
                const bool completed = tractors[i]->tracker()->is_path_completed();
                const float avg_error = (error_samples[i] > 0) ? (total_errors[i] / error_samples[i]) : 0.0f;
                std::cout << "  Robot " << i << ": " << (completed ? "completed" : "running")
                          << " avg_error=" << avg_error << "m" << std::endl;
            }
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

    std::cout << "\n=== MPPI Stress Results (30 Robots) ===" << std::endl;
    std::cout << "Completed: " << total_completed << "/" << kNumRobots << " ("
              << (100.0f * total_completed / kNumRobots) << "%)" << std::endl;
    std::cout << "Average max error: " << (total_max_error / kNumRobots) << "m" << std::endl;
    std::cout << "Average tracking error: " << (total_avg_error / kNumRobots) << "m" << std::endl;

    return 0;
}
