#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "drivekit/pred/mppi.hpp"
#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== MPPI Husky Differential Drive Test ===" << std::endl;

    auto rec = std::make_shared<rerun::RecordingStream>("mppi_diff_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    try {
        auto husky_info =
            fs::Loader::load_from_json("examples/machines/husky.json", concord::Pose{{0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}});
        simulator.add_robot(husky_info);
        std::cout << "Loaded differential-drive husky at (0, 0)" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Failed to load husky: " << e.what() << std::endl;
        return 1;
    }

    auto &husky = simulator.get_robot(0);
    husky.tracker->set_controller_type(drivekit::TrackerType::MPPI);

    auto *mppi_controller = dynamic_cast<drivekit::pred::MPPIFollower *>(husky.tracker->get_controller());
    if (!mppi_controller) {
        std::cerr << "Failed to get MPPI controller" << std::endl;
        return 1;
    }

    auto mppi_config = mppi_controller->get_mppi_config();
    mppi_config.horizon_steps = 25;
    mppi_config.dt = 0.1;
    mppi_config.num_samples = 2000;
    mppi_config.temperature = 0.1;
    mppi_config.steering_noise = 0.15;
    mppi_config.acceleration_noise = 0.1;
    mppi_config.ref_velocity = 0.6; // Normal driving velocity (turn_first will adjust when needed)
    mppi_config.weight_cte = 200.0;
    mppi_config.weight_epsi = 180.0;
    mppi_config.weight_vel = 1.0; // Normal velocity weight (turn_first will adjust when needed)
    mppi_config.weight_steering = 80.0;
    mppi_config.weight_acceleration = 20.0;

    // Turn-first behavior thresholds (hysteresis prevents oscillation)
    mppi_config.turn_first_activation_deg = 70.0; // Activate turn-in-place when heading error > 60°
    mppi_config.turn_first_release_deg = 15.0;    // Resume forward motion when heading error < 15°

    mppi_controller->set_mppi_config(mppi_config);

    // "3" shape path (smaller) extended with sharp zigzag turns
    std::vector<concord::Point> extended_path = {// "3" shape - two curves on right side
                                                 {0.0f, 0.0f},
                                                 {3.0f, 0.0f},
                                                 // First curve (upper)
                                                 {6.0f, 1.0f},
                                                 {9.0f, 3.0f},
                                                 {12.0f, 5.0f},
                                                 {15.0f, 7.0f},
                                                 {18.0f, 9.0f},
                                                 {21.0f, 10.0f},
                                                 {24.0f, 9.0f},
                                                 {27.0f, 7.0f},
                                                 {30.0f, 5.0f},
                                                 // Middle transition
                                                 {33.0f, 5.0f},
                                                 // Second curve (lower - mirror of first)
                                                 {36.0f, 7.0f},
                                                 {39.0f, 9.0f},
                                                 {42.0f, 10.0f},
                                                 {45.0f, 9.0f},
                                                 {48.0f, 7.0f},
                                                 {51.0f, 5.0f},
                                                 {54.0f, 3.0f},
                                                 {57.0f, 1.0f},
                                                 {60.0f, 0.0f},
                                                 // Straight extension to zigzag
                                                 {63.0f, 0.0f},
                                                 // Extended with sharp zigzag turns
                                                 // First sharp turn (will use turn_first=true)
                                                 {68.0f, 10.0f},
                                                 {73.0f, 20.0f},
                                                 // Second sharp turn (will use turn_first=true)
                                                 {78.0f, 20.0f},
                                                 {83.0f, 10.0f},
                                                 {88.0f, 0.0f},
                                                 // Third sharp turn (will use turn_first=false)
                                                 {93.0f, 0.0f},
                                                 {98.0f, -10.0f},
                                                 {103.0f, -20.0f},
                                                 // Fourth sharp turn (will use turn_first=false)
                                                 {108.0f, -20.0f},
                                                 {113.0f, -10.0f},
                                                 {118.0f, 0.0f},
                                                 // Fifth sharp turn (will use turn_first=false)
                                                 {123.0f, 0.0f},
                                                 {128.0f, 10.0f},
                                                 {133.0f, 20.0f},
                                                 // Final straight
                                                 {138.0f, 20.0f}};

    drivekit::PathGoal path(extended_path, 2.0f, 2.0f, false);
    husky.tracker->set_path(path);
    husky.tracker->smoothen(25.0f);

    std::cout << "Starting MPPI path following on the '3' shape with zigzag turns..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f;

    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;

    // Start with turn_first enabled
    husky.state.turn_first = true;

    while (!husky.tracker->is_path_completed()) {
        simulator.tick(dt);
        simulator.tock(5);

        auto status = mppi_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        // Update turn_first based on position along path
        // "3" shape (x < 64): turn_first=true (turn then go)
        // First two sharp zigzag turns (64 <= x < 91): turn_first=false (smooth)
        // Last three sharp zigzag turns (x >= 91): turn_first=true (turn then go)
        auto pos = husky.get_position();
        if (pos.point.x < 85.0f) {
            husky.state.turn_first = true;
        } else {
            husky.state.turn_first = false;
        }

        if (step_count % 60 == 0) {
            double linear_vel, angular_vel;
            husky.get_velocity(linear_vel, angular_vel);
            std::cout << "Step " << step_count / 60 << "s: "
                      << "Pos(" << pos.point.x << ", " << pos.point.y << "), "
                      << "Yaw=" << pos.angle.yaw << ", "
                      << "LinVel=" << linear_vel << ", "
                      << "AngVel=" << angular_vel << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "HeadErr=" << (status.heading_error * 180.0 / M_PI) << "deg, "
                      << "turn_first=" << (husky.state.turn_first ? "ON" : "OFF") << std::endl;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== MPPI Husky Figure-8 + Zigzag Results ===" << std::endl;
    if (husky.tracker->is_path_completed()) {
        std::cout << "Path completed successfully." << std::endl;
    } else {
        std::cout << "Path not completed." << std::endl;
    }

    std::cout << "Max CTE: " << max_cte << " m" << std::endl;
    std::cout << "Avg CTE: " << (cte_samples > 0 ? total_cte / cte_samples : 0.0f) << " m" << std::endl;
    auto final_pos = husky.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
