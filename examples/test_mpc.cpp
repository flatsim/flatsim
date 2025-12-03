#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== MPC (Model Predictive Control) Path Following Test ===" << std::endl;

#ifdef HAS_MPC
    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("mpc_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

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

    // Test MPC Controller with a challenging S-curve path
    std::cout << "\n--- Testing MPC Controller with S-Curve Path ---" << std::endl;
    std::cout << "MPC uses Model Predictive Control with IPOPT optimization" << std::endl;
    std::cout << "This should show optimal trajectory planning with smooth control" << std::endl;

    // Set controller type to MPC
    std::cout << "Setting controller to MPC..." << std::endl;
    tractor.tracker->set_controller_type(navcon::TrackerType::MPC);

    // Access the MPC controller directly to configure it
    auto mpc_controller = dynamic_cast<navcon::pred::MPCFollower *>(tractor.tracker->get_controller());
    if (mpc_controller) {
        auto mpc_config = mpc_controller->get_mpc_config();

        // Configure MPC parameters (balanced lookahead and smoothness)
        mpc_config.horizon_steps = 22; // Moderate prediction horizon (~1.2s)
        mpc_config.dt = 0.1;           // Time step (seconds)
        mpc_config.ref_velocity = 0.8; // Reference normalized speed (~80% throttle)

        // Retuned cost weights for this tractor + flatsim dynamics
        mpc_config.weight_cte = 1500.0;             // Cross-track error
        mpc_config.weight_epsi = 1300.0;            // Heading error
        mpc_config.weight_vel = 0.5;                // Velocity tracking
        mpc_config.weight_steering = 20.0;          // Steering effort (discourage large angles)
        mpc_config.weight_acceleration = 10.0;      // Acceleration effort
        mpc_config.weight_steering_rate = 600.0;    // Steering smoothness
        mpc_config.weight_acceleration_rate = 30.0; // Acceleration smoothness

        mpc_config.max_solver_time = 0.5; // IPOPT solver time limit
        mpc_config.print_level = 0;       // Silent IPOPT output

        mpc_controller->set_mpc_config(mpc_config);

        std::cout << "MPC Configuration:" << std::endl;
        std::cout << "  Horizon: " << mpc_config.horizon_steps << " steps ("
                  << (mpc_config.horizon_steps * mpc_config.dt) << " seconds)" << std::endl;
        std::cout << "  Time step: " << mpc_config.dt << " seconds" << std::endl;
        std::cout << "  Reference velocity: " << mpc_config.ref_velocity << " m/s" << std::endl;
        std::cout << "  CTE weight: " << mpc_config.weight_cte << std::endl;
        std::cout << "  Heading error weight: " << mpc_config.weight_epsi << std::endl;
    } else {
        std::cerr << "Failed to cast to MPC controller!" << std::endl;
        return 1;
    }

    // Create a challenging S-curve path for MPC
    // MPC should handle this optimally by predicting future path curvature
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

    // Smoothen the path for better MPC performance
    std::cout << "Smoothening path with 25cm intervals..." << std::endl;
    tractor.tracker->smoothen(25.0f); // Add points every 25cm for smoother reference trajectory

    std::cout << "Starting MPC path following..." << std::endl;
    std::cout << "Watch for:" << std::endl;
    std::cout << "  - Optimal trajectory planning" << std::endl;
    std::cout << "  - Smooth control actions" << std::endl;
    std::cout << "  - Predictive behavior on curves" << std::endl;
    std::cout << "  - Minimal cross-track error" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;

    while (!tractor.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress every 2 seconds to see path following behavior
        if (step_count % 120 == 0) { // Every ~2 seconds at 60 FPS
            auto target = tractor.tracker->get_current_target();
            auto pos = tractor.get_position();
            auto status = mpc_controller->get_status();

            std::cout << "Step " << step_count / 60 << "s: "
                      << "Robot(" << pos.point.x << "," << pos.point.y << "), "
                      << "Yaw=" << pos.angle.yaw << ", "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "Heading Error=" << (status.heading_error * 180.0 / M_PI) << "deg" << std::endl;
        }

        // Visualize MPC predicted trajectory and lookahead tip for debugging
        if (step_count % 30 == 0) { // ~2 Hz
            const auto &predicted = mpc_controller->get_predicted_trajectory();
            if (!predicted.empty()) {
                // Use 3D visuals like navcon tracker for consistency and visibility
                std::vector<rerun::components::Position3D> pts3d;
                pts3d.reserve(predicted.size());
                for (const auto &p : predicted) {
                    pts3d.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.05f});
                }
                rec->log("mpc/predicted_path", rerun::LineStrips3D({rerun::components::LineStrip3D(pts3d)})
                                                   .with_colors({rerun::Color(0, 255, 0, 128)})
                                                   .with_radii({0.05f}));

                // Lookahead tip (end of horizon) and line from robot to tip
                const auto &tip = predicted.back();
                auto pos = tractor.get_position().point;
                rec->log("mpc/lookahead_tip",
                         rerun::Points3D({{static_cast<float>(tip.x), static_cast<float>(tip.y), 0.1f}})
                             .with_colors({rerun::Color(255, 0, 0, 220)})
                             .with_radii({0.18f}));

                std::vector<rerun::components::Position3D> lookahead_line = {
                    {static_cast<float>(pos.x), static_cast<float>(pos.y), 0.1f},
                    {static_cast<float>(tip.x), static_cast<float>(tip.y), 0.1f}};
                rec->log("mpc/lookahead_line", rerun::LineStrips3D({rerun::components::LineStrip3D(lookahead_line)})
                                                   .with_colors({rerun::Color(255, 255, 0, 220)})
                                                   .with_radii({0.08f}));

                // Draw a lookahead circle at current pose radius = distance to tip (ensure visible)
                const float dx = static_cast<float>(tip.x - pos.x);
                const float dy = static_cast<float>(tip.y - pos.y);
                const float radius = std::max(2.0f, std::sqrt(dx * dx + dy * dy));
                std::vector<rerun::components::Position3D> circle_pts;
                const int segments = 48;
                for (int i = 0; i <= segments; ++i) {
                    float a = 2.0f * static_cast<float>(M_PI) * i / segments;
                    circle_pts.push_back({static_cast<float>(pos.x + radius * std::cos(a)),
                                          static_cast<float>(pos.y + radius * std::sin(a)), 0.05f});
                }
                rec->log("mpc/lookahead_circle", rerun::LineStrips3D({rerun::components::LineStrip3D(circle_pts)})
                                                     .with_colors({rerun::Color(255, 0, 0, 90)})
                                                     .with_radii({0.06f}));
            }
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\n✅ MPC successfully completed the S-curve path!" << std::endl;
        std::cout << "Check Rerun visualization to see the optimal trajectory planning." << std::endl;
        std::cout << "Statistics:" << std::endl;
        std::cout << "  Total time: " << step_count / 60.0f << " seconds" << std::endl;
    } else {
        std::cout << "\n❌ MPC did not complete the path within timeout." << std::endl;
    }

    auto final_pos = tractor.get_position();
    std::cout << "Final position: (" << final_pos.point.x << ", " << final_pos.point.y << ")" << std::endl;

    // Additional challenging test: Figure-8 path
    std::cout << "\n--- Testing MPC with Figure-8 Path Challenge ---" << std::endl;
    std::cout << "This tests MPC's ability to handle tight curves and direction reversals" << std::endl;

    // Create a figure-8 path starting from current position
    std::vector<concord::Point> figure8_path;
    float center_x = final_pos.point.x + 20.0f;
    float center_y = final_pos.point.y;
    float radius = 10.0f;

    // First loop (clockwise)
    for (int i = 0; i <= 36; i++) {
        float angle = i * M_PI / 18.0f; // 10 degree increments
        figure8_path.push_back({center_x + radius * std::cos(angle), center_y + radius * std::sin(angle)});
    }

    // Second loop (counter-clockwise) - creates the figure-8
    for (int i = 0; i <= 36; i++) {
        float angle = M_PI - i * M_PI / 18.0f;
        figure8_path.push_back({center_x + radius * std::cos(angle), center_y - radius * std::sin(angle)});
    }

    navcon::PathGoal figure8_goal(figure8_path, 2.0f, 1.5f, false);
    tractor.tracker->set_path(figure8_goal);
    tractor.tracker->smoothen(20.0f); // Dense smoothing for tight curves

    // Adjust MPC for tighter curves: reuse base config but slow down
    auto tight_config = mpc_controller->get_mpc_config();
    tight_config.ref_velocity = 0.4; // Slightly slower for figure-8
    mpc_controller->set_mpc_config(tight_config);

    std::cout << "Starting Figure-8 navigation (adjusted for tight curves)..." << std::endl;

    start_time = std::chrono::steady_clock::now();
    step_count = 0;

    while (!tractor.tracker->is_path_completed()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 300) { // 5 minute timeout for figure-8
            std::cout << "Figure-8 test timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Print progress for figure-8 test
        if (step_count % 120 == 0) { // Every 2 seconds
            auto target = tractor.tracker->get_current_target();
            auto pos = tractor.get_position();
            auto status = mpc_controller->get_status();

            std::cout << "Figure-8 " << step_count / 60 << "s: "
                      << "Robot(" << pos.point.x << "," << pos.point.y << "), "
                      << "CTE=" << status.cross_track_error << "m" << std::endl;
        }

        // Visualize MPC predicted trajectory and lookahead tip for debugging (figure-8)
        if (step_count % 30 == 0) {
            const auto &predicted = mpc_controller->get_predicted_trajectory();
            if (!predicted.empty()) {
                std::vector<rerun::components::Position3D> pts3d;
                pts3d.reserve(predicted.size());
                for (const auto &p : predicted) {
                    pts3d.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.05f});
                }
                rec->log("mpc/predicted_path", rerun::LineStrips3D({rerun::components::LineStrip3D(pts3d)})
                                                   .with_colors({rerun::Color(0, 255, 0, 128)})
                                                   .with_radii({0.05f}));

                const auto &tip = predicted.back();
                auto pos = tractor.get_position().point;
                rec->log("mpc/lookahead_tip",
                         rerun::Points3D({{static_cast<float>(tip.x), static_cast<float>(tip.y), 0.1f}})
                             .with_colors({rerun::Color(255, 0, 0, 220)})
                             .with_radii({0.18f}));

                std::vector<rerun::components::Position3D> lookahead_line = {
                    {static_cast<float>(pos.x), static_cast<float>(pos.y), 0.1f},
                    {static_cast<float>(tip.x), static_cast<float>(tip.y), 0.1f}};
                rec->log("mpc/lookahead_line", rerun::LineStrips3D({rerun::components::LineStrip3D(lookahead_line)})
                                                   .with_colors({rerun::Color(255, 255, 0, 220)})
                                                   .with_radii({0.08f}));

                const float dx = static_cast<float>(tip.x - pos.x);
                const float dy = static_cast<float>(tip.y - pos.y);
                const float radius = std::max(2.0f, std::sqrt(dx * dx + dy * dy));
                std::vector<rerun::components::Position3D> circle_pts;
                const int segments = 48;
                for (int i = 0; i <= segments; ++i) {
                    float a = 2.0f * static_cast<float>(M_PI) * i / segments;
                    circle_pts.push_back({static_cast<float>(pos.x + radius * std::cos(a)),
                                          static_cast<float>(pos.y + radius * std::sin(a)), 0.05f});
                }
                rec->log("mpc/lookahead_circle", rerun::LineStrips3D({rerun::components::LineStrip3D(circle_pts)})
                                                     .with_colors({rerun::Color(255, 0, 0, 90)})
                                                     .with_radii({0.06f}));
            }
        }
        step_count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\n✅ MPC successfully completed the Figure-8 test!" << std::endl;
        std::cout << "MPC demonstrated optimal control on complex curved trajectories." << std::endl;
        std::cout << "Statistics:" << std::endl;
        std::cout << "  Total time: " << step_count / 60.0f << " seconds" << std::endl;
    } else {
        std::cout << "\n❌ MPC did not complete Figure-8 test within timeout." << std::endl;
    }

    std::cout << "\n=== MPC Controller Test Complete ===" << std::endl;
    std::cout << "MPC characteristics observed:" << std::endl;
    std::cout << "- Optimal trajectory planning considering future path" << std::endl;
    std::cout << "- Smooth, continuous control actions" << std::endl;
    std::cout << "- Predictive behavior anticipating curves" << std::endl;
    std::cout << "- Excellent cross-track error minimization" << std::endl;
    std::cout << "- Higher computational cost (IPOPT optimization)" << std::endl;
    std::cout << "- Best suited for high-performance autonomous navigation" << std::endl;

    return 0;

#else
    std::cerr << "❌ MPC controller not available!" << std::endl;
    std::cerr << "MPC requires:" << std::endl;
    std::cerr << "  - Eigen3 library" << std::endl;
    std::cerr << "  - IPOPT solver" << std::endl;
    std::cerr << "  - CppAD library" << std::endl;
    std::cerr << "\nTo enable MPC:" << std::endl;
    std::cerr << "  1. Install dependencies (see README)" << std::endl;
    std::cerr << "  2. Rebuild with: make reconfig" << std::endl;
    std::cerr << "  3. CMake will auto-detect and enable MPC" << std::endl;
    return 1;
#endif
}
