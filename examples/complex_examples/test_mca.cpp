// MCA (Monte Carlo Approximation / DRA-MPPI) Risk-Aware Path Following Test
//
// Run:
//   ./build/test_mca

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

static std::vector<datapod::Point> corridor_path(float x0, float x1, float step, float y = 0.0f) {
    std::vector<datapod::Point> path;
    if (step <= 0.0f) {
        return path;
    }
    const int n = static_cast<int>((x1 - x0) / step) + 1;
    if (n > 0) {
        path.reserve(static_cast<size_t>(n));
    }
    for (float x = x0; x <= x1 + 1e-6f; x += step) {
        path.push_back({x, y});
    }
    return path;
}

int main() {
    std::cout << "=== MCA (DRA-MPPI) Risk-Aware Path Following Test ===" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    datapod::Pose spawn_pose = utils::make_pose_2d(0.0, 0.0, -1.5708f);
    auto &tractor = sim.spawn_agent("machines/urdf/tractor.urdf", spawn_pose, "mca_0");
    std::cout << "Tractor loaded: " << tractor.name() << " (" << tractor.uuid() << ")\n";

    tractor.tracker()->set_controller_type(drivekit::TrackerType::MCA);
    tractor.set_tracker_enabled(true);

    auto *mca = dynamic_cast<drivekit::pred::MCAFollower *>(tractor.tracker()->get_controller());
    if (!mca) {
        std::cerr << "[Error] Failed to cast to MCA controller\n";
        return 1;
    }

    auto mca_config = mca->get_mca_config();
    mca_config.horizon_steps = 25;
    mca_config.dt = 0.2;
    mca_config.num_samples = 400;
    mca_config.num_mc_samples = 2000;
    mca_config.temperature = 0.3;
    mca_config.steering_noise = 0.4;
    mca_config.acceleration_noise = 0.3;
    mca_config.ref_velocity = 3.0;

    mca_config.weight_cte = 50.0;
    mca_config.weight_epsi = 50.0;
    mca_config.weight_vel = 1.0;
    mca_config.weight_steering = 30.0;
    mca_config.weight_acceleration = 10.0;

    mca_config.weight_soft_risk = 800.0;
    mca_config.weight_hard_risk = 1e7;
    mca_config.risk_threshold = 0.01;

    mca_config.min_velocity_scale = 0.2;
    mca_config.risk_slowdown_gain = 6.0;
    mca_config.robot_radius_margin = 1.0;
    mca->set_mca_config(mca_config);

    std::cout << "MCA Config: horizon=" << mca_config.horizon_steps << " dt=" << mca_config.dt
              << " samples=" << mca_config.num_samples << " mc=" << mca_config.num_mc_samples
              << " risk=" << (mca_config.risk_threshold * 100.0) << "%\n";

    const auto path_pts = corridor_path(0.0f, 50.0f, 0.5f, 0.0f);
    tractor.tracker()->set_path(drivekit::PathGoal(path_pts, 1.0f, 2.0f, false));

    // Obstacles
    sim.world().add_obstacle(types::StaticObstacle{1, datapod::Point{15.0, 0.0, 0.0}, 0.6, 0.1});
    sim.world().add_obstacle(types::StaticObstacle{2, datapod::Point{30.0, 0.8, 0.0}, 0.6, 0.1});
    sim.world().add_obstacle(types::StaticObstacle{3, datapod::Point{45.0, -0.8, 0.0}, 0.6, 0.1});

    sim.world().add_obstacle(
        types::DynamicObstacle{1, datapod::Point{20.0, -4.0, 0.0}, datapod::Point{0.0, 0.6, 0.0}, 0.4, 0.3, 10.0, false});
    sim.world().add_obstacle(
        types::DynamicObstacle{2, datapod::Point{40.0, 0.0, 0.0}, datapod::Point{-0.8, 0.0, 0.0}, 0.4, 0.3, 15.0, false});
    sim.world().add_obstacle(
        types::DynamicObstacle{3, datapod::Point{35.0, 4.0, 0.0}, datapod::Point{0.0, -0.6, 0.0}, 0.4, 0.3, 10.0, false});

    std::cout << "World: " << sim.world().static_obstacles().size() << " static, "
              << sim.world().dynamic_obstacles().size() << " dynamic obstacles\n";

    std::cout << "\nStarting MCA path following...\n" << std::endl;

    float dt = 0.1f;
    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!tractor.tracker()->is_path_completed()) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_s > 300) {
            std::cout << "Timeout reached\n";
            break;
        }

        const auto pos = tractor.get_position();
        sim.world().update_obstacles(dt, pos.point.x, pos.point.y);

        sim.tick(dt);
        sim.tock();

        const auto status = mca->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += static_cast<float>(status.cross_track_error);
        cte_samples++;

        if (step_count % 20 == 0) {
            std::cout << (step_count / 10) << "s: Pos(" << pos.point.x << "," << pos.point.y << ") "
                      << "Yaw=" << utils::get_yaw(pos) << " "
                      << "CTE=" << status.cross_track_error << "m "
                      << "HeadErr=" << (status.heading_error * 180.0 / M_PI) << "deg\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n=== MCA Results ===" << std::endl;
    std::cout << "Completed: " << (tractor.tracker()->is_path_completed() ? "yes" : "no") << std::endl;
    std::cout << "Total time: " << (step_count / 10.0f) << "s" << std::endl;
    std::cout << "Max CTE: " << max_cte << "m" << std::endl;
    std::cout << "Avg CTE: " << (cte_samples > 0 ? (total_cte / static_cast<float>(cte_samples)) : 0.0f) << "m\n";
    const auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")\n";

    return 0;
}
