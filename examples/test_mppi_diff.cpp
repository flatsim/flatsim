#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "drivekit/pred/mppi.hpp"
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

    auto turn_config = mppi_controller->get_mppi_config();
    turn_config.horizon_steps = 25;
    turn_config.dt = 0.1;
    turn_config.num_samples = 2000;
    turn_config.temperature = 0.1;
    turn_config.steering_noise = 0.15;
    turn_config.acceleration_noise = 0.1;
    turn_config.ref_velocity = 0.0; // Hold still while rotating to align
    turn_config.weight_cte = 200.0;
    turn_config.weight_epsi = 180.0;
    turn_config.weight_vel = 60.0; // Heavily penalize linear motion during the turn
    turn_config.weight_steering = 80.0;
    turn_config.weight_acceleration = 20.0;

    auto drive_config = turn_config;
    drive_config.ref_velocity = 0.6;  // Resume forward motion once aligned
    drive_config.weight_vel = 1.0;    // Allow linear motion

    mppi_controller->set_mppi_config(turn_config);

    std::vector<concord::Point> s_curve_path = {
        {0.0f, 0.0f}, {5.0f, 0.0f},  {10.0f, 1.0f},  {15.0f, 3.0f},   {20.0f, 6.0f},  {25.0f, 10.0f},
        {30.0f, 14.0f}, {35.0f, 17.0f}, {40.0f, 19.0f}, {45.0f, 20.0f},  {50.0f, 19.0f}, {55.0f, 17.0f},
        {60.0f, 14.0f}, {65.0f, 10.0f}, {70.0f, 6.0f},  {75.0f, 3.0f},   {80.0f, 1.0f},  {85.0f, 0.0f},
        {90.0f, 0.0f}};

    drivekit::PathGoal path(s_curve_path, 2.0f, 2.0f, false);
    husky.tracker->set_path(path);
    husky.tracker->smoothen(25.0f);

    std::cout << "Starting MPPI path following on the S-curve path..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f;

    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;
    bool using_drive_config = false;
    const float heading_threshold = 0.15f;

    while (!husky.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) {
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        auto status = mppi_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        float heading = std::abs(status.heading_error);
        if (heading < heading_threshold && !using_drive_config) {
            mppi_controller->set_mppi_config(drive_config);
            using_drive_config = true;
        } else if (heading >= heading_threshold && using_drive_config) {
            mppi_controller->set_mppi_config(turn_config);
            using_drive_config = false;
        }

        if (step_count % 120 == 0) {
            auto pos = husky.get_position();
            std::cout << "Step " << step_count / 60 << "s: "
                      << "Robot(" << pos.point.x << ", " << pos.point.y << "), "
                      << "Yaw=" << pos.angle.yaw << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "Heading Err=" << (status.heading_error * 180.0 / M_PI) << "deg" << std::endl;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== MPPI Husky S-Curve Results ===" << std::endl;
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
