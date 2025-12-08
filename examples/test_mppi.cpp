#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== MPPI (Model Predictive Path Integral) Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("mppi_test", "space");
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
        // Spawn tractor at path start, pointing in +X direction (yaw=0)
        auto tractor_info = fs::Loader::load_from_json(
            "examples/machines/tractor.json",
            concord::Pose{
                concord::Point{0.0f, 0.0f},
                concord::Euler{0.0f, 0.0f, -1.5708f}}); // -90 deg to compensate for tractor's default orientation
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;

    // Test MPPI Controller with a challenging S-curve path
    std::cout << "\n--- Testing MPPI Controller with S-Curve Path ---" << std::endl;
    std::cout << "MPPI uses sampling-based optimal control (no external solver needed)" << std::endl;
    std::cout << "This should show smooth trajectory planning with stochastic sampling" << std::endl;

    // Set controller type to MPPI
    std::cout << "Setting controller to MPPI..." << std::endl;
    tractor.tracker->set_controller_type(navcon::TrackerType::MPPI);

    // Access the MPPI controller directly to configure it
    auto mppi_controller = dynamic_cast<navcon::pred::MPPIFollower *>(tractor.tracker->get_controller());
    if (mppi_controller) {
        auto mppi_config = mppi_controller->get_mppi_config();

        // Configure MPPI parameters
        mppi_config.horizon_steps = 25;       // Prediction horizon steps
        mppi_config.dt = 0.1;                 // Time step (seconds)
        mppi_config.num_samples = 2000;       // More samples = smoother control
        mppi_config.temperature = 0.1;        // Lower temperature = more greedy/stable
        mppi_config.steering_noise = 0.15;    // Reduced noise for less oscillation
        mppi_config.acceleration_noise = 0.1; // Reduced acceleration noise
        mppi_config.ref_velocity = 0.8;       // Reference normalized speed (~80% throttle)

        // Cost weights - increased steering penalty to reduce oscillation
        mppi_config.weight_cte = 200.0;         // Cross-track error
        mppi_config.weight_epsi = 180.0;        // Heading error
        mppi_config.weight_vel = 1.0;           // Velocity tracking
        mppi_config.weight_steering = 80.0;     // Higher steering penalty = smoother turns
        mppi_config.weight_acceleration = 20.0; // Higher acceleration penalty = smoother speed

        mppi_controller->set_mppi_config(mppi_config);

        std::cout << "MPPI Configuration:" << std::endl;
        std::cout << "  Horizon: " << mppi_config.horizon_steps << " steps ("
                  << (mppi_config.horizon_steps * mppi_config.dt) << " seconds)" << std::endl;
        std::cout << "  Time step: " << mppi_config.dt << " seconds" << std::endl;
        std::cout << "  Number of samples: " << mppi_config.num_samples << std::endl;
        std::cout << "  Temperature: " << mppi_config.temperature << std::endl;
        std::cout << "  Reference velocity: " << mppi_config.ref_velocity << " m/s" << std::endl;
        std::cout << "  CTE weight: " << mppi_config.weight_cte << std::endl;
        std::cout << "  Heading error weight: " << mppi_config.weight_epsi << std::endl;
    } else {
        std::cerr << "Failed to cast to MPPI controller!" << std::endl;
        return 1;
    }

    // Create a challenging S-curve path for MPPI
    // MPPI should handle this by sampling many trajectories and selecting the best
    std::vector<concord::Point> s_curve_path = {
        {0.0f, 0.0f},   // Start
        {5.0f, 0.0f},   // Straight section
        {10.0f, 1.0f},  // Begin curve
        {15.0f, 3.0f},  //
        {20.0f, 6.0f},  //
        {25.0f, 10.0f}, // Peak of first curve
        {30.0f, 14.0f}, //
        {35.0f, 17.0f}, //
        {40.0f, 19.0f}, //
        {45.0f, 20.0f}, // Transition
        {50.0f, 19.0f}, // S-curve begins
        {55.0f, 17.0f}, //
        {60.0f, 14.0f}, //
        {65.0f, 10.0f}, // Bottom of S
        {70.0f, 6.0f},  //
        {75.0f, 3.0f},  //
        {80.0f, 1.0f},  //
        {85.0f, 0.0f},  // Straight section
        {90.0f, 0.0f}   // End
    };

    navcon::PathGoal path(s_curve_path, 2.0f, 2.0f, false); // Reasonable tolerance

    std::cout << "Setting navigation path with " << s_curve_path.size() << " waypoints..." << std::endl;
    tractor.tracker->set_path(path);

    // Smoothen the path for better MPPI performance
    std::cout << "Smoothening path with 25cm intervals..." << std::endl;
    tractor.tracker->smoothen(25.0f); // Add points every 25cm for smoother reference trajectory

    std::cout << "Starting MPPI path following..." << std::endl;
    std::cout << "Watch for:" << std::endl;
    std::cout << "  - Stochastic trajectory sampling" << std::endl;
    std::cout << "  - Smooth control actions" << std::endl;
    std::cout << "  - Adaptive behavior on curves" << std::endl;
    std::cout << "  - Good cross-track error tracking" << std::endl;

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

        // Track error statistics
        auto status = mppi_controller->get_status();
        max_cte = std::max(max_cte, static_cast<float>(status.cross_track_error));
        total_cte += status.cross_track_error;
        cte_samples++;

        // Print progress every 2 seconds to see path following behavior
        if (step_count % 120 == 0) { // Every ~2 seconds at 60 FPS
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
        std::cout << "\nMPPI successfully completed the S-curve path!" << std::endl;
        std::cout << "Check Rerun visualization to see the trajectory planning." << std::endl;
    } else {
        std::cout << "\nMPPI did not complete the path within timeout." << std::endl;
    }

    std::cout << "\nStatistics:" << std::endl;
    std::cout << "  Total time: " << step_count / 60.0f << " seconds" << std::endl;
    std::cout << "  Max cross-track error: " << max_cte << " m" << std::endl;
    std::cout << "  Avg cross-track error: " << (cte_samples > 0 ? total_cte / cte_samples : 0.0f) << " m" << std::endl;

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    return 0;
}
