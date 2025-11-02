#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

#include "flatsim/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "rerun/recording_stream.hpp"

std::string generate_uuid() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    static std::uniform_int_distribution<> dis2(8, 11);

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

std::vector<concord::Point> generate_u_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<concord::Point> path;
    for (int i = 0; i <= 20; ++i) {
        float t = i / 20.0f;
        float angle = M_PI * t;
        float x = offset_x + 15.0f * scale * std::sin(angle);
        float y = offset_y - 15.0f * scale * std::cos(angle) + 15.0f * scale;
        path.push_back({x, y});
    }
    return path;
}

std::vector<concord::Point> generate_o_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<concord::Point> path;
    for (int i = 0; i <= 24; ++i) {
        float t = i / 24.0f;
        float angle = 2.0f * M_PI * t;
        float x = offset_x + 15.0f * scale * std::cos(angle);
        float y = offset_y + 15.0f * scale * std::sin(angle);
        path.push_back({x, y});
    }
    return path;
}

std::vector<concord::Point> generate_l_shape(float offset_x, float offset_y, float scale = 1.0f) {
    std::vector<concord::Point> path;
    for (int i = 0; i <= 10; ++i) {
        float y = offset_y + i * 3.0f * scale;
        path.push_back({offset_x, y});
    }
    for (int i = 1; i <= 10; ++i) {
        float x = offset_x + i * 3.0f * scale;
        path.push_back({x, offset_y + 30.0f * scale});
    }
    return path;
}

int main(int argc, char *argv[]) {
    std::cout << "=== PID Controller Path Following Test ===" << std::endl;

    auto rec = std::make_shared<rerun::RecordingStream>("pid_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    std::vector<concord::Point> spawn_positions = {{-50.0f, -50.0f}, {50.0f, -50.0f}, {-50.0f, 50.0f}, {50.0f, 50.0f}};

    std::vector<pigment::RGB> colors = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}};

    try {
        for (int i = 0; i < 4; ++i) {
            auto tractor_info = fs::Loader::load_from_json(
                "examples/machines/tractor.json", concord::Pose{spawn_positions[i], concord::Euler{0.0f, 0.0f, 0.0f}},
                "tractor" + std::to_string(i), colors[i]);

            tractor_info.uuid = generate_uuid();
            tractor_info.seqid = tractor_info.type + "_" + std::to_string(i);

            simulator.add_robot(tractor_info);
            std::cout << "Loaded tractor " << i << " (seqid: " << tractor_info.seqid << ") at (" << spawn_positions[i].x
                      << ", " << spawn_positions[i].y << ")" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractors: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "All 4 tractors loaded successfully\n" << std::endl;

    std::vector<std::vector<concord::Point>> paths = {
        generate_s_shape(spawn_positions[0].x, spawn_positions[0].y, 1.0f),
        generate_u_shape(spawn_positions[1].x, spawn_positions[1].y, 1.0f),
        generate_o_shape(spawn_positions[2].x, spawn_positions[2].y, 1.0f),
        generate_l_shape(spawn_positions[3].x, spawn_positions[3].y, 1.0f)};

    std::vector<std::string> shape_names = {"S-shape", "U-shape", "O-shape", "L-shape"};

    for (int i = 0; i < 4; ++i) {
        auto &tractor = simulator.get_robot(i);
        tractor.navcon->set_controller_type(navcon::NavconControllerType::PID);
        navcon::PathGoal path(paths[i], 2.0f, 2.5f, false);
        tractor.navcon->set_path(path);
        std::cout << "Tractor " << i << " following " << shape_names[i] << " (" << paths[i].size() << " waypoints)"
                  << std::endl;
    }

    std::cout << "\nStarting PID path following for all 4 tractors...\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f;

    int step_count = 0;
    std::vector<float> max_errors(4, 0.0f);
    std::vector<float> total_errors(4, 0.0f);
    std::vector<int> error_samples(4, 0);

    bool all_completed = false;
    while (!all_completed) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 180) {
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        all_completed = true;
        for (int i = 0; i < 4; ++i) {
            auto &tractor = simulator.get_robot(i);
            if (!tractor.navcon->is_path_completed()) {
                all_completed = false;

                auto target = tractor.navcon->get_current_target();
                auto pos = tractor.get_position();
                float tracking_error =
                    std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

                max_errors[i] = std::max(max_errors[i], tracking_error);
                total_errors[i] += tracking_error;
                error_samples[i]++;
            }
        }

        if (step_count % 120 == 0) {
            std::cout << "\n--- Progress at " << step_count / 60 << "s ---" << std::endl;
            for (int i = 0; i < 4; ++i) {
                auto &tractor = simulator.get_robot(i);
                auto pos = tractor.get_position();
                bool completed = tractor.navcon->is_path_completed();
                float avg_error = error_samples[i] > 0 ? total_errors[i] / error_samples[i] : 0.0f;
                std::cout << "Tractor " << i << " (" << shape_names[i]
                          << "): " << (completed ? "✅ COMPLETED" : "🚜 Running") << " | Pos(" << pos.point.x << ","
                          << pos.point.y << ")" << " | Avg Error: " << avg_error << "m" << std::endl;
            }
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== PID Controller Test Results ===" << std::endl;
    for (int i = 0; i < 4; ++i) {
        auto &tractor = simulator.get_robot(i);
        bool completed = tractor.navcon->is_path_completed();
        float avg_error = error_samples[i] > 0 ? total_errors[i] / error_samples[i] : 0.0f;

        std::cout << "\nTractor " << i << " (" << shape_names[i] << "):" << std::endl;
        std::cout << "  Status: " << (completed ? "✅ Completed" : "❌ Not completed") << std::endl;
        std::cout << "  Max error: " << max_errors[i] << "m" << std::endl;
        std::cout << "  Avg error: " << avg_error << "m" << std::endl;
        auto final_pos = tractor.get_position();
        std::cout << "  Final pos: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;
    }

    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "All tractors followed different shaped paths simultaneously" << std::endl;

    return 0;
}
