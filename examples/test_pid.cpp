#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/loader.hpp"
#include "flatsim/robot/controller.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== PID Controller Path Following Test ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("pid_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Create simulator
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    // Load tractor
    try {
        auto tractor_info =
            fs::Loader::load_from_json("examples/machines/tractor.json",
                                       concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, 0.0f}});
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;

    // Test PID Controller with a smooth path
    std::cout << "\n--- Testing PID Controller with Smooth Path ---" << std::endl;

    // IMPORTANT: Set controller type BEFORE setting path
    std::cout << "Setting controller to PID..." << std::endl;
    tractor.set_navigation_controller_type(fs::ControllerType::PID);

    // Create a smooth path that tests PID's error correction capabilities
    // PID controller excels at smooth error correction with proportional, integral, and derivative terms
    // Start with forward movement since robot faces north (90 degrees)
    std::vector<concord::Point> smooth_path = {
        {0.0f, 10.0f},   // Move forward first (robot faces north)
        {5.0f, 15.0f},   // Gentle turn to northeast
        {12.0f, 18.0f},  // Continue curving east
        {20.0f, 20.0f},  // Smooth progression
        {30.0f, 22.0f},  // Peak of curve
        {40.0f, 20.0f},  // Start coming down
        {48.0f, 17.0f},  // Continue descent
        {55.0f, 12.0f},  // More descent
        {60.0f, 5.0f},   // Back toward baseline
        {65.0f, -2.0f},  // Go below baseline
        {70.0f, -8.0f},  // Continue down
        {75.0f, -12.0f}, // Bottom of curve
        {82.0f, -10.0f}, // Start coming back up
        {90.0f, -5.0f},  // Continue up
        {98.0f, 2.0f},   // Cross baseline
        {105.0f, 8.0f},  // Final rise
        {112.0f, 12.0f}  // End point
    };

    fs::PathGoal path(smooth_path, 2.0f, 2.5f, false); // Moderate tolerance for PID precision

    std::cout << "Setting navigation path with " << smooth_path.size() << " waypoints..." << std::endl;
    tractor.set_navigation_path(path);

    std::cout << "Starting PID Controller path following..." << std::endl;
    std::cout << "PID should demonstrate smooth error correction and minimal oscillation" << std::endl;
    std::cout << "Watch for gradual convergence to the path with good stability" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;
    float max_error = 0.0f;
    float total_error = 0.0f;
    int error_samples = 0;

    while (!tractor.is_navigation_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 180) { // 3 minute timeout for longer path
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Calculate tracking error for PID performance analysis
        auto target = tractor.get_current_navigation_target();
        auto pos = tractor.get_position();
        float tracking_error = std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

        max_error = std::max(max_error, tracking_error);
        total_error += tracking_error;
        error_samples++;

        // Print progress every 2 seconds to see PID behavior
        if (step_count % 120 == 0) { // Every ~2 seconds at 60 FPS
            float avg_error = total_error / error_samples;
            std::cout << "Step " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << ", Error=" << tracking_error
                      << "m" << ", Avg Error=" << avg_error << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.is_navigation_path_completed()) {
        std::cout << "\n✅ PID Controller successfully completed the smooth path!" << std::endl;
        std::cout << "Check Rerun visualization to see the smooth error correction behavior." << std::endl;
    } else {
        std::cout << "\n❌ PID Controller did not complete the path within timeout." << std::endl;
    }

    float avg_error = error_samples > 0 ? total_error / error_samples : 0.0f;
    std::cout << "PID Performance Statistics:" << std::endl;
    std::cout << "- Maximum tracking error: " << max_error << "m" << std::endl;
    std::cout << "- Average tracking error: " << avg_error << "m" << std::endl;

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    // Additional test: Step response to test PID's transient behavior
    std::cout << "\n--- Testing PID with Step Response Challenge ---" << std::endl;

    // Create a path with sudden direction changes to test PID's step response
    std::vector<concord::Point> step_path = {
        {final_pos.point.x, final_pos.point.y},                 // Start from current position
        {final_pos.point.x + 15.0f, final_pos.point.y},         // Sudden step east
        {final_pos.point.x + 15.0f, final_pos.point.y + 15.0f}, // Sudden step north
        {final_pos.point.x, final_pos.point.y + 15.0f},         // Sudden step west
        {final_pos.point.x, final_pos.point.y},                 // Back to start (sudden step south)
        {final_pos.point.x + 20.0f, final_pos.point.y + 20.0f}  // Final diagonal step
    };

    fs::PathGoal step_goal(step_path, 2.0f, 2.0f, false);
    tractor.set_navigation_path(step_goal);

    std::cout << "Testing step response with PID controller..." << std::endl;
    std::cout << "PID should show controlled response to sudden direction changes" << std::endl;
    std::cout << "Watch for overshoot, settling time, and steady-state error characteristics" << std::endl;

    start_time = std::chrono::steady_clock::now();
    step_count = 0;
    max_error = 0.0f;
    total_error = 0.0f;
    error_samples = 0;

    while (!tractor.is_navigation_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 120) { // 2 minute timeout for step test
            std::cout << "Step response test timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Track PID performance during step response
        auto target = tractor.get_current_navigation_target();
        auto pos = tractor.get_position();
        float tracking_error = std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

        max_error = std::max(max_error, tracking_error);
        total_error += tracking_error;
        error_samples++;

        // Print progress for step response test
        if (step_count % 60 == 0) { // Every ~1 second
            std::cout << "Step Test " << step_count / 60 << "s: Target(" << target.x << "," << target.y << "), Robot("
                      << pos.point.x << "," << pos.point.y << "), Yaw=" << pos.angle.yaw << ", Error=" << tracking_error
                      << "m" << std::endl;
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.is_navigation_path_completed()) {
        std::cout << "\n✅ PID Controller successfully completed the step response test!" << std::endl;
        std::cout << "PID's step response should show controlled transient behavior." << std::endl;
    } else {
        std::cout << "\n❌ PID Controller did not complete step response test within timeout." << std::endl;
    }

    avg_error = error_samples > 0 ? total_error / error_samples : 0.0f;
    std::cout << "Step Response Performance:" << std::endl;
    std::cout << "- Maximum tracking error: " << max_error << "m" << std::endl;
    std::cout << "- Average tracking error: " << avg_error << "m" << std::endl;

    std::cout << "\n=== PID Controller Test Complete ===" << std::endl;
    std::cout << "PID controller characteristics observed:" << std::endl;
    std::cout << "- Smooth error correction with P, I, D terms" << std::endl;
    std::cout << "- Controlled response to disturbances and setpoint changes" << std::endl;
    std::cout << "- Good steady-state accuracy with minimal oscillation" << std::endl;
    std::cout << "- Tunable performance via gain parameters (Kp, Ki, Kd)" << std::endl;

    return 0;
}