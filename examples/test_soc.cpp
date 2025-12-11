#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

// Build an S-shaped path similar to xtra/navcon/examples/soc.cpp
static std::vector<concord::Point> build_s_shape_path() {
    return {{0.0f, 0.0f},   {2.0f, 0.0f},   {4.0f, 0.5f},   {6.0f, 1.5f},   {8.0f, 3.0f},   {10.0f, 5.0f},
            {12.0f, 7.5f},  {14.0f, 10.0f}, {16.0f, 12.0f}, {18.0f, 13.5f}, {20.0f, 14.5f}, {22.0f, 15.0f},
            {24.0f, 15.0f}, {26.0f, 15.0f}, {28.0f, 15.0f}, {30.0f, 15.0f}, {32.0f, 15.0f}, {34.0f, 15.0f},
            {36.0f, 14.5f}, {38.0f, 13.5f}, {40.0f, 12.0f}, {42.0f, 10.0f}, {44.0f, 7.5f},  {46.0f, 5.0f},
            {48.0f, 3.0f},  {50.0f, 1.5f},  {52.0f, 0.5f},  {54.0f, 0.0f},  {56.0f, 0.0f}};
}

int main(int argc, char *argv[]) {
    std::cout << "=== SOC (Stochastic Optimal Control) Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("soc_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // Create simulator
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    // Load tractor - spawn at first waypoint
    try {
        // Spawn tractor at path start, pointing along +X (adjust yaw if needed)
        auto tractor_info = fs::Loader::load_from_json(
            "examples/machines/tractor.json",
            concord::Pose{concord::Point{0.0f, 0.0f},
                          concord::Euler{0.0f, 0.0f, -1.5708f}}); // -90 deg to match tractor's default orientation
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;

    std::cout << "\n--- Testing SOC Controller with S-Curve Path ---" << std::endl;
    std::cout << "SOC wraps MPPI with guided sampling (stochastic optimal control)" << std::endl;

    // Set controller type to SOC
    std::cout << "Setting controller to SOC..." << std::endl;
    tractor.tracker->set_controller_type(drivekit::TrackerType::SOC);

    // Access the SOC controller directly to configure it
    auto soc_controller = dynamic_cast<drivekit::pred::SOCFollower *>(tractor.tracker->get_controller());
    if (soc_controller) {
        auto soc_config = soc_controller->get_soc_config();

        // Configuration tuned for smoother behavior in Flatsim.
        // Start from the MPPI example (examples/test_mppi.cpp) and keep it conservative.
        soc_config.horizon_steps = 25; // Longer horizon for look-ahead
        soc_config.dt = 0.1;
        soc_config.guide_samples = 64; // Currently unused in core MPPI mapping
        soc_config.num_samples = 2000; // More samples = smoother control
        soc_config.temperature = 0.1;  // Lower temperature = more stable/less noisy
        soc_config.guide_temperature = 1.0;
        soc_config.steering_noise = 0.15;    // Lower steering noise to reduce jitter
        soc_config.acceleration_noise = 0.1; // Lower accel noise for smoother speed
        soc_config.initial_steer_variance = 0.1;
        soc_config.min_steer_variance = 1e-4;
        soc_config.max_steer_variance = 0.5;
        soc_config.weight_cte = 200.0;  // Cross-track error
        soc_config.weight_epsi = 180.0; // Heading error
        soc_config.weight_vel = 1.0;
        soc_config.weight_steering = 80.0;     // Higher steering penalty = smoother turns
        soc_config.weight_acceleration = 20.0; // Higher accel penalty = smoother speed
        soc_config.ref_velocity = 0.6;         // Slower nominal speed to avoid overshoot
        soc_config.svgd_iterations = 2;
        soc_config.svgd_step_size = 0.2;

        soc_controller->set_soc_config(soc_config);

        std::cout << "SOC Configuration:" << std::endl;
        std::cout << "  Horizon: " << soc_config.horizon_steps << " steps ("
                  << (soc_config.horizon_steps * soc_config.dt) << " seconds)" << std::endl;
        std::cout << "  Time step: " << soc_config.dt << " seconds" << std::endl;
        std::cout << "  Guide samples: " << soc_config.guide_samples << std::endl;
        std::cout << "  Samples: " << soc_config.num_samples << std::endl;
        std::cout << "  Temperature: " << soc_config.temperature << std::endl;
        std::cout << "  Guide temperature: " << soc_config.guide_temperature << std::endl;
        std::cout << "  Reference velocity: " << soc_config.ref_velocity << " m/s" << std::endl;
        std::cout << "  CTE weight: " << soc_config.weight_cte << std::endl;
        std::cout << "  Heading error weight: " << soc_config.weight_epsi << std::endl;
    } else {
        std::cerr << "Failed to cast to SOC controller!" << std::endl;
        return 1;
    }

    // Create S-curve path for SOC
    auto s_curve_path = build_s_shape_path();
    drivekit::PathGoal path(s_curve_path, 2.0f, 2.0f, false); // Reasonable tolerances

    std::cout << "Setting navigation path with " << s_curve_path.size() << " waypoints..." << std::endl;
    tractor.tracker->set_path(path);

    // Smooth path for better SOC performance
    std::cout << "Smoothening path with 25cm intervals..." << std::endl;
    tractor.tracker->smoothen(25.0f);

    std::cout << "Starting SOC path following..." << std::endl;
    std::cout << "Watch for:" << std::endl;
    std::cout << "  - Guided stochastic trajectory sampling" << std::endl;
    std::cout << "  - Smooth control actions" << std::endl;
    std::cout << "  - Adaptive behavior on curves" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    float max_cte = 0.0f;
    float total_cte = 0.0f;
    int cte_samples = 0;

    while (!tractor.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Track error statistics via SOC status (inherits MPPI status)
        auto status = soc_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += status.cross_track_error;
        cte_samples++;

        // Print progress every ~2 seconds
        if (step_count % 120 == 0) {
            auto target = tractor.tracker->get_current_target();
            auto pos = tractor.get_position();

            std::cout << "Step " << step_count / 60 << "s: "
                      << "Robot(" << pos.point.x << "," << pos.point.y << "), "
                      << "Yaw=" << pos.angle.yaw << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "Heading Error=" << (status.heading_error * 180.0 / M_PI) << "deg" << std::endl;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\nSOC successfully completed the S-curve path!" << std::endl;
        std::cout << "Check Rerun visualization to see the guided trajectory planning." << std::endl;
    } else {
        std::cout << "\nSOC did not complete the path within timeout." << std::endl;
    }

    std::cout << "\nStatistics:" << std::endl;
    std::cout << "  Total time: " << step_count / 60.0f << " seconds" << std::endl;
    std::cout << "  Max cross-track error: " << max_cte << " m" << std::endl;
    std::cout << "  Avg cross-track error: " << (cte_samples > 0 ? total_cte / cte_samples : 0.0f) << " m" << std::endl;

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
