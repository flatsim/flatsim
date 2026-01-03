// Field GPS/IMU + NMEA output demo (LOCAL mode - single process)
//
// Migrated from `examples_old/test_field_gps_nmea.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_field_gps_nmea_local

#include "flatsim/utils.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "flatsim/agent.hpp"
#include "flatsim/agent/sensor/gps_sensor.hpp"
#include "flatsim/agent/sensor/imu_sensor.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include "pigment/pigment.hpp"

#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/turners/dubins.hpp"
#include "flatsim/utils.hpp"

// Different colors for each robot
static const std::vector<pigment::RGB> ROBOT_COLORS = {
    pigment::RGB{90, 196, 185}, // #5AC4B9 - Turquoise
    pigment::RGB{90, 153, 196}, // #5A99C4 - Sky Blue
    pigment::RGB{90, 101, 196}, // #5A65C4 - Periwinkle
};

// Generate smooth path with Dubins curves between swath endpoints
static std::vector<datapod::Point>
generate_dubins_path(const std::vector<std::shared_ptr<const farmtrax::Swath>> &swaths, float turning_radius,
                     float step_size = 0.5f) {
    std::vector<datapod::Point> path;
    if (swaths.empty()) return path;

    farmtrax::turners::Dubins dubins(turning_radius);

    std::vector<std::shared_ptr<const farmtrax::Swath>> working_swaths;
    for (const auto &swath : swaths) {
        if (swath->type == farmtrax::SwathType::Swath) {
            working_swaths.push_back(swath);
        }
    }
    if (working_swaths.empty()) return path;

    for (size_t i = 0; i < working_swaths.size(); ++i) {
        const auto &swath = working_swaths[i];
        path.push_back(swath->line.start);
        path.push_back(swath->line.end);

        if (i + 1 < working_swaths.size()) {
            const auto &next_swath = working_swaths[i + 1];

            const float dx_curr = swath->line.end.x - swath->line.start.x;
            const float dy_curr = swath->line.end.y - swath->line.start.y;
            const float yaw_end = std::atan2(dy_curr, dx_curr);

            const float dx_next = next_swath->line.end.x - next_swath->line.start.x;
            const float dy_next = next_swath->line.end.y - next_swath->line.start.y;
            const float yaw_start = std::atan2(dy_next, dx_next);

            farmtrax::turners::Pose2D start_pose(swath->line.end, yaw_end);
            farmtrax::turners::Pose2D end_pose(next_swath->line.start, yaw_start);

            auto dubins_path = dubins.plan_path(start_pose, end_pose, step_size);
            for (size_t j = 1; j + 1 < dubins_path.waypoints.size(); ++j) {
                path.push_back(dubins_path.waypoints[j].point);
            }
        }
    }

    return path;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    std::cout << "=== Field GPS/IMU + NMEA Demo (LOCAL mode) ===\n";

    const int num_machines = 1; // This migrated demo keeps the original default.
    std::cout << "Number of machines: " << num_machines << "\n";

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    // Create a large square field that leaves only 1/6th of world as border.
    // World is 500x500 centered at 0,0 (so -250 to +250).
    // Border: 500/6 ~= 83m => field spans roughly [-167, +167].
    datapod::Polygon poly;
    poly.vertices.push_back(datapod::Point{-167.0, -167.0, 0.0});
    poly.vertices.push_back(datapod::Point{167.0, -167.0, 0.0});
    poly.vertices.push_back(datapod::Point{167.0, 167.0, 0.0});
    poly.vertices.push_back(datapod::Point{-167.0, 167.0, 0.0});
    poly.vertices.push_back(datapod::Point{-167.0, -167.0, 0.0});

    farmtrax::Field field(poly, datum, true, 100000.0);
    field.gen_field(18.0, 90.0, 0);

    const auto &part = field.get_parts()[0];
    auto part_area = part.boundary.polygon.area();
    std::cout << "\nUsing part 0:\n";
    std::cout << "Field area: " << std::fixed << std::setprecision(1) << part_area << " sq.m (" << (part_area / 10000.0)
              << " hectares)\n";
    std::cout << "Headlands: " << part.headlands.size() << ", Swaths: " << part.swaths.size() << "\n";

    auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[0]);
    farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
    divy.compute_division();
    auto &res = divy.result();

    float turning_radius = 5.0f;
    std::vector<datapod::Point> path;

    // Load tractors and assign paths (kept single-machine by default).
    if (!res.swaths_per_machine.empty() && !res.swaths_per_machine.at(0).empty()) {
        farmtrax::Nety nety(res.swaths_per_machine.at(0));
        nety.field_traversal();
        const auto &swaths = nety.get_swaths();
        path = generate_dubins_path(swaths, turning_radius, 0.5f);
    }

    if (path.size() < 2) {
        std::cerr << "[Error] Generated path is empty/too small\n";
        return 1;
    }

    float spawn_x = path[0].x;
    float spawn_y = path[0].y;
    float spawn_yaw = std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);

    const std::string uuid = "field_gps_tractor_0";
    const auto color = ROBOT_COLORS[0];

    auto &tractor = sim.spawn_agent("examples/machines/tractor.json",
                                    utils::make_pose_2d(spawn_x, spawn_y, spawn_yaw - 1.5708f), uuid, color);

    std::cout << "Robot UUID: " << tractor.uuid() << "\n";
    std::cout << "GPS SHM path: /dev/shm/flatsim_" << tractor.uuid() << "_GPS\n";
    std::cout << "IMU SHM path: /dev/shm/flatsim_" << tractor.uuid() << "_IMU\n";

    // Add sensors (SensorManager auto-enables SHM output once robot uuid is set).
    tractor.machine().sensors.add(std::make_unique<fs::GPSSensor>(10.0, true, 3.0, 0.02));
    tractor.machine().sensors.add(std::make_unique<fs::IMUSensor>(100.0, 0.01, 0.001, 0.1));

    tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MPPI);
    tractor.controls().tracker().set_enabled(true);
    tractor.set_navigation_enabled(true);

    if (auto *mppi = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker()->get_controller())) {
        auto mppi_cfg = mppi->get_mppi_config();
        mppi_cfg.horizon_steps = 20;
        mppi_cfg.dt = 0.1;
        mppi_cfg.num_samples = 1000;
        mppi_cfg.temperature = 0.1;
        mppi_cfg.steering_noise = 0.15;
        mppi_cfg.acceleration_noise = 0.1;
        mppi_cfg.ref_velocity = 0.6;
        mppi_cfg.weight_cte = 200.0;
        mppi_cfg.weight_epsi = 180.0;
        mppi_cfg.weight_vel = 1.0;
        mppi_cfg.weight_steering = 80.0;
        mppi_cfg.weight_acceleration = 20.0;
        mppi->set_mppi_config(mppi_cfg);
    }

    tractor.tracker()->set_path(drivekit::PathGoal(path, 2.0f, 2.0f, false));
    tractor.tracker()->smoothen(25.0f);

    std::cout << "\n*** GPS will output continuously - Press Ctrl+C to stop ***\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    const float dt = 0.016f;
    int step_count = 0;
    int last_reset_step = -1000;
    bool phtg = false;

    while (true) {
        auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();

        sim.tick(dt);
        sim.tock();

        // Toggle PHTG status every 10 seconds (600 steps at 60 FPS)
        if (step_count % 600 == 0) {
            auto *gps_sensor = tractor.machine().sensors.get<fs::GPSSensor>();
            if (gps_sensor) {
                phtg = !phtg;
                gps_sensor->set_phtg_status(phtg);
                std::cout << "\n*** PHTG status toggled to: " << (phtg ? "ENABLED" : "DISABLED") << " ***\n"
                          << std::endl;
            }
        }

        // Dynamic path update: restart when near completion (second-to-last waypoint).
        const auto pos = tractor.get_position();
        const auto second_to_last = path[path.size() - 2];
        const float dx = static_cast<float>(pos.point.x) - second_to_last.x;
        const float dy = static_cast<float>(pos.point.y) - second_to_last.y;
        const float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 3.0f && (step_count - last_reset_step) > 100) {
            std::cout << "\n*** At second-to-last waypoint! Restarting field path... ***\n" << std::endl;
            tractor.tracker()->set_path(drivekit::PathGoal(path, 2.0f, 2.0f, false));
            tractor.tracker()->smoothen(25.0f);
            last_reset_step = step_count;
        }

        if (step_count % 300 == 0) {
            std::cout << "\nTime: " << elapsed << "s\n";
            const auto completed = tractor.tracker()->is_path_completed();
            std::cout << "  Machine 0: (" << std::fixed << std::setprecision(1) << pos.point.x << ", " << pos.point.y
                      << ") " << (completed ? "[COMPLETED]" : "[RUNNING]") << "\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}
