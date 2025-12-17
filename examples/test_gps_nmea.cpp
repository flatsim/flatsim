#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/sensor/gps_sensor.hpp"
#include "flatsim/robot/sensor/imu_sensor.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== GPS NMEA Output Test (Endless Loop) ===" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("gps_nmea_test", "space");
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

    // Load tractor
    try {
        auto tractor_info =
            fs::Loader::load_from_json("examples/machines/tractor.json",
                                       concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, -1.5708f}});
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);

    // Override UUID with a memorable name for easy testing
    tractor.info.uuid = "test_gps_nmea";

    std::cout << "Tractor loaded: " << tractor.info.name << " (UUID: " << tractor.info.uuid << ")" << std::endl;

    // Add GPS sensor
    auto gps = std::make_unique<fs::GPSSensor>(10.0, true, 3.0, 0.02); // 10Hz, RTK enabled
    tractor.sensors.add(std::move(gps));
    std::cout << "Added GPS sensor to tractor (10Hz, RTK enabled)" << std::endl;
    std::cout << "GPS NMEA output: /dev/shm/flatsim_" << tractor.info.uuid << "_GPS" << std::endl;
    std::cout << "GPS format file: /tmp/flatsim_" << tractor.info.uuid << "/GPS.format" << std::endl;

    // Add IMU sensor
    auto imu = std::make_unique<fs::IMUSensor>(100.0, 0.01, 0.001, 0.1); // 100Hz, realistic noise
    tractor.sensors.add(std::move(imu));
    std::cout << "\nAdded IMU sensor to tractor (100Hz, 9-DOF)" << std::endl;
    std::cout << "IMU binary output: /dev/shm/flatsim_" << tractor.info.uuid << "_IMU" << std::endl;
    std::cout << "IMU format file: /tmp/flatsim_" << tractor.info.uuid << "/IMU.format" << std::endl;

    // Configure MPPI controller for endless circular path
    std::cout << "\n--- Setting up MPPI controller for endless loop ---" << std::endl;
    tractor.tracker->set_controller_type(drivekit::TrackerType::MPPI);

    auto mppi_controller = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker->get_controller());
    if (mppi_controller) {
        auto mppi_config = mppi_controller->get_mppi_config();

        // Configure MPPI parameters
        mppi_config.horizon_steps = 20;
        mppi_config.dt = 0.1;
        mppi_config.num_samples = 1500;
        mppi_config.temperature = 0.1;
        mppi_config.steering_noise = 0.1;
        mppi_config.acceleration_noise = 0.08;
        mppi_config.ref_velocity = 0.6; // Moderate speed

        // Cost weights
        mppi_config.weight_cte = 150.0;
        mppi_config.weight_epsi = 120.0;
        mppi_config.weight_vel = 1.0;
        mppi_config.weight_steering = 60.0;
        mppi_config.weight_acceleration = 15.0;

        mppi_controller->set_mppi_config(mppi_config);
        std::cout << "MPPI configured for smooth circular driving" << std::endl;
    }

    // Create circular path (endless loop)
    std::vector<concord::Point> circular_path;
    float radius = 30.0f;
    float center_x = 40.0f;
    float center_y = 0.0f;
    int num_points = 72; // 72 points = 5 degree intervals for smooth circle

    for (int i = 0; i < num_points; i++) { // Don't duplicate start/end point
        float angle = (i * 2.0f * M_PI) / num_points;
        circular_path.push_back({center_x + radius * std::cos(angle), center_y + radius * std::sin(angle)});
    }

    drivekit::PathGoal path(circular_path, 2.0f, 3.0f, true); // loop=true for endless

    std::cout << "Setting circular path with radius " << radius << "m..." << std::endl;
    tractor.tracker->set_path(path);
    // DON'T smooth - it breaks the loop closure!
    // tractor.tracker->smoothen(50.0f);

    std::cout << "\nStarting endless loop..." << std::endl;
    std::cout << "GPS will continuously output NMEA sentences to shared memory" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    int step_count = 0;

    bool phtg = false;

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        simulator.tick(dt);
        simulator.tock(5);

        // Get tracker status for debugging
        bool path_completed = tractor.tracker->is_path_completed();
        bool goal_reached = tractor.tracker->is_goal_reached();

        // Toggle PHTG status every 10 seconds (600 steps at 60 FPS)
        if (step_count % 600 == 0) {
            auto *gps_sensor = tractor.sensors.get<fs::GPSSensor>();
            if (gps_sensor) {
                phtg = !phtg;
                gps_sensor->set_phtg_status(phtg);
                std::cout << "\n*** PHTG status toggled to: " << (phtg ? "ENABLED" : "DISABLED") << " ***\n"
                          << std::endl;
            }
        }

        // Print detailed status every 2 seconds
        if (step_count % 120 == 0) { // Every 2 seconds at 60 FPS
            auto pos = tractor.get_position();
            double linear_vel, angular_vel;
            tractor.get_velocity(linear_vel, angular_vel);
            auto *gps_sensor = tractor.sensors.get<fs::GPSSensor>();

            std::cout << "\n=== Time " << elapsed << "s (step " << step_count << ") ===" << std::endl;
            std::cout << "Position: (" << pos.point.x << ", " << pos.point.y << ")" << std::endl;
            std::cout << "Velocity: linear=" << linear_vel << " m/s, angular=" << angular_vel << " rad/s" << std::endl;
            std::cout << "Path completed: " << (path_completed ? "YES" : "NO") << std::endl;
            std::cout << "Goal reached: " << (goal_reached ? "YES" : "NO") << std::endl;

            if (gps_sensor) {
                auto gps_data = gps_sensor->get_gps_data();
                std::cout << "GPS: lat=" << gps_data.latitude << ", lon=" << gps_data.longitude
                          << ", RTK=" << static_cast<int>(gps_data.rtk_status) << ", Sats=" << gps_data.num_satellites
                          << ", PHTG=" << (phtg ? "ON" : "OFF") << std::endl;
            }

            auto *imu_sensor = tractor.sensors.get<fs::IMUSensor>();
            if (imu_sensor) {
                auto imu_data = imu_sensor->get_imu_data();
                std::cout << "IMU: accel=(" << imu_data.accel_x << "," << imu_data.accel_y << "," << imu_data.accel_z
                          << ") m/s²"
                          << ", gyro=(" << imu_data.gyro_x << "," << imu_data.gyro_y << "," << imu_data.gyro_z
                          << ") rad/s"
                          << ", yaw=" << imu_data.yaw << " rad" << std::endl;
            }
        }

        // Dynamic path update: Check if we're at the second-to-last waypoint
        auto pos = tractor.get_position();
        auto second_to_last = circular_path[circular_path.size() - 2];
        float dist =
            std::sqrt(std::pow(pos.point.x - second_to_last.x, 2) + std::pow(pos.point.y - second_to_last.y, 2));

        // When we reach second-to-last waypoint, send new path
        if (dist < 3.0f) { // Within tolerance of second-to-last waypoint
            static int last_reset_step = -1000;
            if (step_count - last_reset_step > 100) { // Avoid spamming (only reset every ~1.6 seconds)
                std::cout << "\n*** At second-to-last waypoint! Sending new path... ***\n" << std::endl;
                drivekit::PathGoal new_path(circular_path, 2.0f, 3.0f, false);
                tractor.tracker->set_path(new_path);
                last_reset_step = step_count;
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    return 0;
}
