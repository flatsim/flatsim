// MPPI Differential Drive Path Following Test
//
// Run:
//   ./build/test_mppi_diff

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <drivekit.hpp>
#include <iostream>
#include <thread>
#include <vector>

int main() {
    std::cout << "=== MPPI Differential Drive Path Following Test ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    auto &husky = sim.spawn_agent("machines/urdf/husky.urdf", utils::make_pose_2d(0.0, 0.0, 0.0), "mppi_diff_0");
    std::cout << "Husky loaded: " << husky.name() << " (" << husky.uuid() << ")\n";

    // Configure MPPI controller
    husky.tracker()->set_controller_type(drivekit::TrackerType::MPPI);
    husky.set_tracker_enabled(true);

    auto *mppi = dynamic_cast<drivekit::pred::MPPIFollower *>(husky.tracker()->get_controller());
    if (!mppi) {
        std::cerr << "[Error] Failed to get MPPI controller\n";
        return 1;
    }

    auto mppi_config = mppi->get_mppi_config();
    mppi_config.horizon_steps = 25;
    mppi_config.dt = 0.1;
    mppi_config.num_samples = 2000;
    mppi_config.temperature = 0.1;
    mppi_config.steering_noise = 0.15;
    mppi_config.acceleration_noise = 0.1;
    mppi_config.ref_velocity = 0.6;
    mppi_config.weight_cte = 200.0;
    mppi_config.weight_epsi = 180.0;
    mppi_config.weight_vel = 1.0;
    mppi_config.weight_steering = 80.0;
    mppi_config.weight_acceleration = 20.0;
    mppi_config.turn_first_activation_deg = 70.0;
    mppi_config.turn_first_release_deg = 15.0;
    mppi->set_mppi_config(mppi_config);

    std::cout << "MPPI Config: horizon=" << mppi_config.horizon_steps << " samples=" << mppi_config.num_samples
              << " ref_v=" << mppi_config.ref_velocity << "\n";

    std::vector<datapod::Point> path_points = {
        {0.0f, 0.0f},   {3.0f, 0.0f},   {6.0f, 1.0f},   {9.0f, 3.0f},   {12.0f, 5.0f},  {15.0f, 7.0f},
        {18.0f, 9.0f},  {21.0f, 10.0f}, {24.0f, 9.0f},  {27.0f, 7.0f},  {30.0f, 5.0f},  {33.0f, 5.0f},
        {36.0f, 7.0f},  {39.0f, 9.0f},  {42.0f, 10.0f}, {45.0f, 9.0f},  {48.0f, 7.0f},  {51.0f, 5.0f},
        {54.0f, 3.0f},  {57.0f, 1.0f},  {60.0f, 0.0f},  {63.0f, 0.0f},  {68.0f, 10.0f}, {73.0f, 20.0f},
        {78.0f, 20.0f}, {83.0f, 10.0f}, {88.0f, 0.0f},  {93.0f, 0.0f},  {98.0f, -10.0f},
        {103.0f, -20.0f}, {108.0f, -20.0f}, {113.0f, -10.0f}, {118.0f, 0.0f}, {123.0f, 0.0f},
        {128.0f, 10.0f}, {133.0f, 20.0f}, {138.0f, 20.0f}};

    husky.tracker()->set_path(drivekit::PathGoal(path_points, 2.0f, 2.0f, false));
    husky.tracker()->smoothen(25.0f);

    std::cout << "Path set with " << path_points.size() << " waypoints\n";
    std::cout << "Starting MPPI diff drive path following...\n" << std::endl;

    const float dt = 0.016f;
    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!husky.tracker()->is_path_completed()) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 300) {
            std::cout << "Timeout reached\n";
            break;
        }

        sim.tick(dt);
        if (step_count % 5 == 0) {
            sim.tock();
        }

        const auto status = mppi->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        if (step_count % 60 == 0) {
            const auto pose = husky.get_position();
            std::cout << (step_count / 60) << "s: Pos(" << pose.point.x << "," << pose.point.y << ") "
                      << "Yaw=" << utils::get_yaw(pose) << " "
                      << "CTE=" << status.cross_track_error << "m "
                      << "HeadErr=" << (status.heading_error * 180.0 / M_PI) << "deg\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== MPPI Diff Results ===" << std::endl;
    std::cout << "Completed: " << (husky.tracker()->is_path_completed() ? "yes" : "no") << std::endl;
    std::cout << "Max CTE: " << max_cte << "m" << std::endl;
    std::cout << "Avg CTE: " << (cte_samples > 0 ? (total_cte / static_cast<float>(cte_samples)) : 0.0f) << "m\n";
    const auto final_pose = husky.get_position();
    std::cout << "Final position: (" << final_pose.point.x << ", " << final_pose.point.y << ")\n";

    return 0;
}
