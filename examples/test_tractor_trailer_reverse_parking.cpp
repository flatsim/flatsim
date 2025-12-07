#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== Tractor-Trailer Reverse Parking with MPC ===" << std::endl;
    std::cout << "Inspired by the MPC-based truck-trailer maneuvers in xtra/truck-trailer-amr" << std::endl;

#ifdef HAS_MPC
    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("tractor_trailer_reverse_parking", "space");
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

    // ============================================================================
    // SCENARIO: Reverse park tractor+trailer into a tight parking bay
    // ============================================================================

    // Starting position: tractor facing along +X in navcon's frame.
    // Due to the internal 90° offset between the physics chassis and navcon,
    // we spawn with yaw = -90° so navcon sees yaw ≈ 0 (same trick as test_mpc.cpp).
    constexpr float kTractorSpawnYaw = -1.5708f;

    // Starting pose: positioned a bit "in front" of the parking bay
    concord::Pose tractor_start{
        concord::Point{0.0f, 5.0f},
        concord::Euler{0.0f, 0.0f, kTractorSpawnYaw}
    };

    // Approximate target parking position for the trailer (from the paper demo)
    concord::Pose trailer_target{
        concord::Point{-2.0f, -10.0f},
        concord::Euler{0.0f, 0.0f, static_cast<float>(M_PI)} // Facing -X (reversed)
    };

    try {
        // Load tractor at starting position
        auto tractor_info = fs::Loader::load_from_json("examples/machines/tractor.json", tractor_start);
        simulator.add_robot(tractor_info);

        // Place trailer such that its front hitch roughly aligns with the tractor rear hitch.
        // Values come from examples/machines/{tractor,trailer}.json:
        //   tractor rear_hitch.y  = -1.47
        //   trailer front_hitch.y =  2.54
        // We offset the trailer center so these hitches overlap in world coordinates.
        constexpr float kTractorRearHitchOffsetY = -1.47f;
        constexpr float kTrailerFrontHitchOffsetY = 2.54f;
        const float hitch_delta_y = kTractorRearHitchOffsetY - kTrailerFrontHitchOffsetY;

        // Rotate the local hitch offset into world coordinates using the tractor's spawn yaw
        const float cos_yaw = std::cos(tractor_start.angle.yaw);
        const float sin_yaw = std::sin(tractor_start.angle.yaw);
        const float trailer_offset_x = -sin_yaw * hitch_delta_y;
        const float trailer_offset_y = cos_yaw * hitch_delta_y;

        const float trailer_spawn_x = tractor_start.point.x + trailer_offset_x;
        const float trailer_spawn_y = tractor_start.point.y + trailer_offset_y;

        auto trailer_info = fs::Loader::load_from_json(
            "examples/machines/trailer.json",
            concord::Pose{
                concord::Point{trailer_spawn_x, trailer_spawn_y},
                concord::Euler{0.0f, 0.0f, kTractorSpawnYaw}});
        simulator.add_robot(trailer_info);

    } catch (const std::exception &e) {
        std::cerr << "Failed to load vehicles: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    auto &trailer = simulator.get_robot(1);

    std::cout << "Tractor loaded: " << tractor.info.name << std::endl;
    std::cout << "Trailer loaded: " << trailer.info.name << std::endl;

    // Connect tractor and trailer through their hitches so they act as a single articulated vehicle
    std::cout << "Connecting tractor and trailer via hitches..." << std::endl;
    if (!tractor.chain.try_connect_nearby()) {
        std::cerr << "  ⚠️ Failed to automatically connect tractor and trailer. They will move independently.\n";
    } else {
        tractor.chain.print_chain_status();
    }

    // ============================================================================
    // MPC Configuration for trailer-style backing manoeuvre
    // ============================================================================

    std::cout << "\n--- Configuring MPC for Trailer Backing ---" << std::endl;
    std::cout << "This is a challenging problem because:" << std::endl;
    std::cout << "  1. Reverse driving inverts control response" << std::endl;
    std::cout << "  2. Trailer jackknifing must be avoided (beta angle limits)" << std::endl;
    std::cout << "  3. Non-holonomic constraints make it harder" << std::endl;
    std::cout << "  4. Requires long prediction horizon to 'see' backing path" << std::endl;

    tractor.tracker->set_controller_type(navcon::TrackerType::MPC_TRAILER);

    auto mpc_controller = dynamic_cast<navcon::pred::MPCTrailerFollower *>(tractor.tracker->get_controller());
    if (mpc_controller) {
        auto mpc_config = mpc_controller->get_mpc_config();

        // Approximate truck–trailer geometry from machine descriptions
        // Tractor wheelbase (distance between front and rear axle centers)
        double front_y_max = -1e9;
        double rear_y_min = 1e9;
        for (const auto &w : tractor.info.wheels) {
            if (w.pose.point.y >= 0.0f) {
                front_y_max = std::max(front_y_max, static_cast<double>(w.pose.point.y));
            } else {
                rear_y_min = std::min(rear_y_min, static_cast<double>(w.pose.point.y));
            }
        }
        double truck_L0 = (front_y_max > -1e8 && rear_y_min < 1e8) ? std::abs(front_y_max - rear_y_min) : 1.5;

        // Distance from tractor rear axle to hitch (M0)
        double truck_M0 = 0.0;
        auto tractor_hitch_it = tractor.info.hitches.find("rear_hitch");
        if (tractor_hitch_it != tractor.info.hitches.end()) {
            const auto &hp = tractor_hitch_it->second.bound.pose.point;
            // Rear axle approximately at rear_y_min in tractor frame
            truck_M0 = static_cast<double>(hp.y) - rear_y_min;
        }

        // Trailer hitch length: distance from trailer reference point to front hitch
        double trailer_L1 = 2.5;
        auto hitch_it = trailer.info.hitches.find("front_hitch");
        if (hitch_it != trailer.info.hitches.end()) {
            const auto &hp = hitch_it->second.bound.pose.point;
            // Distance from axle (rear wheels) to front hitch along trailer axis
            double axle_y = 0.0;
            for (const auto &w : trailer.info.wheels) {
                axle_y = std::min(axle_y, static_cast<double>(w.pose.point.y));
            }
            trailer_L1 = static_cast<double>(hp.y) - axle_y;
        }

        // Use MPC settings similar to examples/test_mpc.cpp but target REVERSE motion
        mpc_config.horizon_steps = 32; // ≈ 3.2 s with dt=0.1
        mpc_config.dt = 0.1;
        // Negative reference velocity = backing up
        mpc_config.ref_velocity = -0.6;

        // Retuned cost weights (copied from test_mpc)
        mpc_config.weight_cte = 1500.0;
        mpc_config.weight_epsi = 1300.0;
        mpc_config.weight_vel = 0.5;
        mpc_config.weight_steering = 10.0;
        mpc_config.weight_acceleration = 10.0;
        mpc_config.weight_steering_rate = 600.0;
        mpc_config.weight_acceleration_rate = 30.0;

        // Trailer articulation penalty and geometry
        mpc_config.weight_beta = 2500.0;
        mpc_config.truck_wheelbase = truck_L0;
        mpc_config.trailer_hitch_length = trailer_L1;
        mpc_config.truck_hitch_offset = truck_M0;

        mpc_config.max_solver_time = 0.5;
        mpc_config.print_level = 0;

        mpc_controller->set_mpc_config(mpc_config);

        std::cout << "\nMPC Trailer Backing Configuration:" << std::endl;
        std::cout << "  Horizon: " << mpc_config.horizon_steps << " steps ("
                  << (mpc_config.horizon_steps * mpc_config.dt) << " seconds)" << std::endl;
        std::cout << "  Time step: " << mpc_config.dt << " seconds" << std::endl;
        std::cout << "  Reference velocity (reverse): " << mpc_config.ref_velocity << " m/s" << std::endl;
        std::cout << "  CTE weight: " << mpc_config.weight_cte << std::endl;
        std::cout << "  Heading error weight: " << mpc_config.weight_epsi << std::endl;
        std::cout << "  Jackknife prevention (steering_rate weight): " << mpc_config.weight_steering_rate
                  << std::endl;
        std::cout << "  Articulation weight: " << mpc_config.weight_beta << std::endl;
        std::cout << "  Truck wheelbase L0: " << mpc_config.truck_wheelbase << " m" << std::endl;
        std::cout << "  Truck hitch offset M0: " << mpc_config.truck_hitch_offset << " m" << std::endl;
        std::cout << "  Trailer hitch length L1: " << mpc_config.trailer_hitch_length << " m" << std::endl;
    } else {
        std::cerr << "Failed to cast to MPC controller!" << std::endl;
        return 1;
    }

    // ============================================================================
    // Create reverse-parking style path (approx. trailer path from the paper)
    // ============================================================================

    std::cout << "\n--- Creating Reverse Parking Maneuver ---" << std::endl;

    // The path is tracked by the tractor's MPC while the tractor is *reversing*,
    // but the reference is for the TRAILER. Start the path at the trailer's
    // initial position and bend it into a parking bay-like pose.
    auto trailer_start_pose = trailer.get_position();
    const float sx = trailer_start_pose.point.x;
    const float sy = trailer_start_pose.point.y;

    std::vector<concord::Point> reverse_parking_path = {
        // Phase 1: straight back
        {sx, sy},
        {sx - 4.0f, sy},
        {sx - 8.0f, sy + 1.0f},

        // Phase 2: start curving "into" the bay
        {sx - 11.0f, sy + 3.0f},
        {sx - 13.0f, sy + 5.0f},

        // Phase 3: final backed-in position near desired target
        {trailer_target.point.x - 1.0f, trailer_target.point.y + 0.5f},
        {trailer_target.point.x, trailer_target.point.y},
    };

    navcon::PathGoal path(reverse_parking_path, 1.0f, 0.6f, false); // Tight positional tolerance, modest speed

    std::cout << "Setting reverse parking path with " << reverse_parking_path.size() << " waypoints..." << std::endl;
    tractor.tracker->set_path(path);

    // Smoothen for trailer backing (needs very smooth reference)
    std::cout << "Smoothening path with 20cm intervals..." << std::endl;
    tractor.tracker->smoothen(20.0f);

    std::cout << "\nStarting Reverse/Forward Multi-Stage Maneuver..." << std::endl;
    std::cout << "Stage 1: reverse into corner" << std::endl;
    std::cout << "Stage 2: pull forward to refine position" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS
    int step_count = 0;

    bool reverse_phase = true;
    bool forward_phase_started = false;

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 180) { // 3 minute timeout (parking can be slow!)
            std::cout << "Timeout reached!" << std::endl;
            break;
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Monitor path tracking + jackknife angle (beta = theta_tractor - theta_trailer)
        if (step_count % 120 == 0) { // Every ~2 seconds
            auto tractor_pos = tractor.get_position();
            auto trailer_pos = trailer.get_position();
            auto target = tractor.tracker->get_current_target();
            
            float beta_angle = tractor_pos.angle.yaw - trailer_pos.angle.yaw;
            // Normalize angle to [-pi, pi]
            while (beta_angle > M_PI) beta_angle -= 2*M_PI;
            while (beta_angle < -M_PI) beta_angle += 2*M_PI;
            
            auto status = mpc_controller->get_status();

            std::cout << (reverse_phase ? "[REV] " : "[FWD] ")
                      << "Step " << step_count / 60 << "s: "
                      << "Tractor(" << tractor_pos.point.x << "," << tractor_pos.point.y << "), "
                      << "Trailer(" << trailer_pos.point.x << "," << trailer_pos.point.y << "), "
                      << "Target(" << target.x << "," << target.y << "), "
                      << "CTE=" << status.cross_track_error << "m, "
                      << "HeadingErr=" << (status.heading_error * 180.0 / M_PI) << "deg, "
                      << "Beta=" << (beta_angle * 180.0 / M_PI) << "° "
                      << (std::abs(beta_angle) > M_PI / 3 ? "⚠️ JACKKNIFE RISK!" : "✓")
                      << std::endl;
        }

        // Phase switching: when reverse path is done (or close), switch to a
        // short forward refinement phase where we pull the trailer slightly
        // forward to settle near the final parking pose.
        if (reverse_phase && tractor.tracker->is_path_completed()) {
            reverse_phase = false;
            forward_phase_started = true;

            std::cout << "\n--- Reverse phase complete, switching to forward refinement ---" << std::endl;

            // Reconfigure MPC for forward driving
            auto cfg = mpc_controller->get_mpc_config();
            cfg.ref_velocity = 0.5; // modest forward speed
            mpc_controller->set_mpc_config(cfg);

            // Build a short forward path from current trailer position to target
            auto trailer_pose_now = trailer.get_position();
            std::vector<concord::Point> forward_path = {
                trailer_pose_now.point,
                // Small pull-forward arc towards the target
                {trailer_pose_now.point.x + 3.0f, trailer_pose_now.point.y + 1.0f},
                trailer_target.point,
            };

            navcon::PathGoal forward_goal(forward_path, 0.8f, 0.6f, false);
            tractor.tracker->set_path(forward_goal);
            tractor.tracker->smoothen(20.0f);
        }

        // Forward phase finished
        if (!reverse_phase && forward_phase_started && tractor.tracker->is_path_completed()) {
            break;
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    if (tractor.tracker->is_path_completed()) {
        std::cout << "\n🎯 ✅ Successfully completed multi-stage trailer maneuver!" << std::endl;
        std::cout << "MPC solved the challenging trailer backing + forward refinement problem!" << std::endl;
        std::cout << "Statistics:" << std::endl;
        std::cout << "  Total time: " << step_count / 60.0f << " seconds" << std::endl;
        
        auto final_tractor = tractor.get_position();
        auto final_trailer = trailer.get_position();
        std::cout << "  Final tractor: (" << final_tractor.point.x << ", " << final_tractor.point.y << ")" << std::endl;
        std::cout << "  Final trailer: (" << final_trailer.point.x << ", " << final_trailer.point.y << ")" << std::endl;
    } else {
        std::cout << "\n❌ Parking maneuver did not complete within timeout." << std::endl;
    }

    return 0;

#else
    std::cerr << "❌ MPC controller not available!" << std::endl;
    std::cerr << "This example requires MPC (IPOPT + CppAD)" << std::endl;
    std::cerr << "Please rebuild with: make reconfig" << std::endl;
    return 1;
#endif
}
