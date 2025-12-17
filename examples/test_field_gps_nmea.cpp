#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "concord/concord.hpp"
#include "flatsim/core/loader.hpp"
#include "flatsim/robot/sensor/gps_sensor.hpp"
#include "flatsim/robot/sensor/imu_sensor.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "pigment/pigment.hpp"
#include "rerun/recording_stream.hpp"

#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/turners/dubins.hpp"

// Different colors for each robot
const std::vector<pigment::RGB> ROBOT_COLORS = {
    pigment::RGB{90, 196, 185}, // #5AC4B9 - Turquoise
    pigment::RGB{90, 153, 196}, // #5A99C4 - Sky Blue
    pigment::RGB{90, 101, 196}, // #5A65C4 - Periwinkle
    pigment::RGB{90, 196, 185}, // #5AC4B9 (repeat)
    pigment::RGB{90, 153, 196}, // #5A99C4 (repeat)
    pigment::RGB{90, 101, 196}, // #5A65C4 (repeat)
    pigment::RGB{90, 196, 185}, // #5AC4B9 (repeat)
    pigment::RGB{90, 153, 196}, // #5A99C4 (repeat)
};

// Generate smooth path with Dubins curves between swath endpoints
// Only traverse actual Swath types (AB-lines), skip Connection/Around/Headland types
std::vector<concord::Point> generate_dubins_path(const std::vector<std::shared_ptr<const farmtrax::Swath>> &swaths,
                                                 float turning_radius, float step_size = 0.5f) {
    std::vector<concord::Point> path;

    if (swaths.empty()) return path;

    farmtrax::turners::Dubins dubins(turning_radius);

    // Filter to only include actual working swaths (not connections)
    std::vector<std::shared_ptr<const farmtrax::Swath>> working_swaths;
    for (const auto &swath : swaths) {
        if (swath->type == farmtrax::SwathType::Swath) {
            working_swaths.push_back(swath);
        }
    }

    if (working_swaths.empty()) return path;

    for (size_t i = 0; i < working_swaths.size(); ++i) {
        const auto &swath = working_swaths[i];

        // Add start point of swath
        path.push_back(swath->line.getStart());

        // Add end point of swath
        path.push_back(swath->line.getEnd());

        // If there's a next swath, generate Dubins curve to connect them
        if (i + 1 < working_swaths.size()) {
            const auto &next_swath = working_swaths[i + 1];

            // Calculate heading at end of current swath
            float dx_curr = swath->line.getEnd().x - swath->line.getStart().x;
            float dy_curr = swath->line.getEnd().y - swath->line.getStart().y;
            float yaw_end = std::atan2(dy_curr, dx_curr);

            // Calculate heading at start of next swath
            float dx_next = next_swath->line.getEnd().x - next_swath->line.getStart().x;
            float dy_next = next_swath->line.getEnd().y - next_swath->line.getStart().y;
            float yaw_start = std::atan2(dy_next, dx_next);

            // Create poses for Dubins path planning
            concord::Pose start_pose;
            start_pose.point = swath->line.getEnd();
            start_pose.angle.yaw = yaw_end;

            concord::Pose end_pose;
            end_pose.point = next_swath->line.getStart();
            end_pose.angle.yaw = yaw_start;

            // Generate Dubins path for the turn
            auto dubins_path = dubins.plan_path(start_pose, end_pose, step_size);

            // Add Dubins waypoints (skip first and last as they're already in the path)
            for (size_t j = 1; j < dubins_path.waypoints.size() - 1; ++j) {
                path.push_back(dubins_path.waypoints[j].point);
            }
        }
    }

    return path;
}

int main() {
    std::cout << "=== Farmtrax Multi-Machine Field Coverage Test ===" << std::endl;
    std::cout << "Features: Robot colors, Collision avoidance, Dubins curves\n" << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax_test", "space");
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

    // Create a large field that leaves only 1/6th of world as border
    // World is 500x500 centered at 0,0 (so -250 to +250)
    // World border: 500/6 = ~83m, so field goes from -167 to +167 (334m x 334m)
    float world_dim = 500.0f;
    float border = world_dim / 6.0f;                // 1/6th of world = ~83m border
    float field_half = (world_dim - border) / 5.0f; // Half of field size
    concord::Polygon poly;
    poly.addPoint(concord::Point{-field_half, -field_half, 0.0});
    poly.addPoint(concord::Point{field_half, -field_half, 0.0});
    poly.addPoint(concord::Point{field_half, field_half, 0.0});
    poly.addPoint(concord::Point{-field_half, field_half, 0.0});
    poly.addPoint(concord::Point{-field_half, -field_half, 0.0}); // Close the polygon

    std::cout << "Creating square field (" << (field_half * 2) << "m x " << (field_half * 2) << "m) with " << border
              << "m border\n";

    // area_threshold parameter controls field partitioning - set huge to avoid splitting
    farmtrax::Field field(poly, world_datum, true, 999999999.0);

    // Generate field with 36m swath width, 0 degree angle, 2 headland passes
    field.gen_field(12.0, 0.0, 2);

    int num_machines = 1; // Number of tractors
    std::cout << "Number of machines: " << num_machines << "\n";

    auto part_cnt = field.get_parts().size();
    std::cout << "Total field parts: " << part_cnt << "\n";

    // Process the first field part (in a simple square, there should be only one part)
    if (field.get_parts().empty()) {
        std::cerr << "No field parts generated!\n";
        return 1;
    }

    // Print info for all parts
    for (size_t i = 0; i < field.get_parts().size(); ++i) {
        const auto &p = field.get_parts()[i];
        auto area = boost::geometry::area(p.boundary.b_polygon);
        std::cout << "Part " << i << ": area=" << area << " sq.m, headlands=" << p.headlands.size()
                  << ", swaths=" << p.swaths.size() << "\n";
    }

    const auto &part = field.get_parts()[0];
    auto part_area = boost::geometry::area(part.boundary.b_polygon);
    std::cout << "\nUsing part 0:\n";
    std::cout << "Field area: " << std::fixed << std::setprecision(1) << part_area << " sq.m (" << (part_area / 10000.0)
              << " hectares)\n";
    std::cout << "Headlands: " << part.headlands.size() << ", Swaths: " << part.swaths.size() << "\n";

    // Create division for multiple machines
    auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[0]);
    farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
    divy.compute_division();

    auto &res = divy.result();

    // Turning radius for Dubins curves (typical tractor turning radius)
    float turning_radius = 5.0f;

    // Track which machines were successfully loaded
    std::vector<int> active_machines;
    std::vector<concord::Point> path; // Store path outside loop for later reuse

    // Load tractors and assign paths
    for (int m = 0; m < num_machines; ++m) {
        if (res.swaths_per_machine.at(m).empty()) {
            std::cout << "Machine " << m << " has no swaths assigned\n";
            continue;
        }

        std::cout << "\n--- Machine " << m << " ---\n";
        std::cout << "Assigned swaths: " << res.swaths_per_machine.at(m).size() << "\n";

        // Create Nety instance to optimize swath traversal order
        farmtrax::Nety nety(res.swaths_per_machine.at(m));
        nety.field_traversal(); // Reorder swaths for optimal traversal

        const auto &swaths = nety.get_swaths();
        std::cout << "Optimized swaths: " << swaths.size() << "\n";

        // Build path from swaths using Dubins curves for smooth turns
        path = generate_dubins_path(swaths, turning_radius, 0.5f);

        if (path.empty()) {
            std::cout << "Machine " << m << " has empty path after optimization\n";
            continue;
        }

        std::cout << "Generated path with " << path.size() << " waypoints (including Dubins curves)\n";

        // Calculate starting position for this machine
        float spawn_x = path[0].x;
        float spawn_y = path[0].y - (m * 8.0f); // Offset each machine by 8m in Y for safety
        float spawn_yaw = 0.0f;

        // Calculate initial heading towards first waypoint
        if (path.size() > 1) {
            float dx = path[1].x - path[0].x;
            float dy = path[1].y - path[0].y;
            spawn_yaw = std::atan2(dy, dx);
        }

        // Load tractor
        try {
            auto tractor_info = fs::Loader::load_from_json(
                "examples/machines/tractor.json",
                concord::Pose{concord::Point{spawn_x, spawn_y},
                              concord::Euler{0.0f, 0.0f, spawn_yaw - 1.5708f}}); // -90 deg for tractor orientation

            // Override UUID with fixed name BEFORE adding to simulator so sensors use it
            tractor_info.uuid = "field_gps_tractor";

            // Set unique color for this robot
            tractor_info.color = ROBOT_COLORS[m % ROBOT_COLORS.size()];
            for (auto &karo : tractor_info.karos) {
                karo.color = ROBOT_COLORS[m % ROBOT_COLORS.size()];
            }

            simulator.add_robot(tractor_info);
            std::cout << "Loaded tractor " << m << " at (" << spawn_x << ", " << spawn_y << ") with color ("
                      << (int)tractor_info.color.r << ", " << (int)tractor_info.color.g << ", "
                      << (int)tractor_info.color.b << ")\n";
        } catch (const std::exception &e) {
            std::cerr << "Failed to load tractor " << m << ": " << e.what() << std::endl;
            return 1;
        }

        // Set up path following for this tractor
        auto &tractor = simulator.get_robot(m);

        std::cout << "Robot UUID: " << tractor.info.uuid << "\n";
        std::cout << "GPS SHM path: /dev/shm/flatsim_" << tractor.info.uuid << "_GPS\n";

        // Update robot color after loading
        tractor.update_color(ROBOT_COLORS[m % ROBOT_COLORS.size()]);

        // Add GPS sensor (will use the UUID set above)
        auto gps = std::make_unique<fs::GPSSensor>(10.0, true, 3.0, 0.02); // 10Hz, RTK enabled
        tractor.sensors.add(std::move(gps));
        std::cout << "Added GPS sensor to Robot " << m << " (10Hz, RTK enabled)\n";

        // Add IMU sensor
        auto imu = std::make_unique<fs::IMUSensor>(100.0, 0.01, 0.001, 0.1); // 100Hz
        tractor.sensors.add(std::move(imu));
        std::cout << "Added IMU sensor to Robot " << m << " (100Hz, 9-DOF)\n";

        // Configure MPPI controller
        tractor.tracker->set_controller_type(drivekit::TrackerType::MPPI);
        auto mppi_controller = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker->get_controller());

        if (mppi_controller) {
            auto mppi_config = mppi_controller->get_mppi_config();
            mppi_config.horizon_steps = 20;
            mppi_config.dt = 0.1;
            mppi_config.num_samples = 1000;
            mppi_config.temperature = 0.1;
            mppi_config.steering_noise = 0.15;
            mppi_config.acceleration_noise = 0.1;
            mppi_config.ref_velocity = 0.6; // Slower speed for agricultural work
            mppi_config.weight_cte = 200.0;
            mppi_config.weight_epsi = 180.0;
            mppi_config.weight_vel = 1.0;
            mppi_config.weight_steering = 80.0;
            mppi_config.weight_acceleration = 20.0;
            mppi_controller->set_mppi_config(mppi_config);
        }

        // Set path with reasonable tolerance
        drivekit::PathGoal path_goal(path, 2.0f, 2.0f, false);
        tractor.tracker->set_path(path_goal);
        tractor.tracker->smoothen(25.0f); // Add points every 25cm for smoother following

        active_machines.push_back(m);
        std::cout << "Path set with " << path.size() << " waypoints\n";
    }

    std::cout << "\n=== Starting Simulation ===" << std::endl;
    std::cout << "Active machines: " << active_machines.size() << "\n";
    std::cout << "Collision avoidance: Lower priority robots (higher index) will stop when too close\n";

    // Track stopped state for each robot
    std::vector<bool> robot_stopped(num_machines, false);

    // Simulation loop (endless - restarts path when near completion)
    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS
    int step_count = 0;
    int last_reset_step = -1000;
    bool phtg = false;

    std::cout << "\n*** GPS will output continuously - Press Ctrl+C to stop ***\n" << std::endl;

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        simulator.tick(dt);
        simulator.tock(5);

        // Get robot reference
        auto &tractor = simulator.get_robot(0);
        auto pos = tractor.get_position();

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

        // Dynamic path update: Check if we're at the second-to-last waypoint
        auto second_to_last = path[path.size() - 2];
        float dist =
            std::sqrt(std::pow(pos.point.x - second_to_last.x, 2) + std::pow(pos.point.y - second_to_last.y, 2));

        // When we reach second-to-last waypoint, send new path
        if (dist < 3.0f) {                            // Within tolerance of second-to-last waypoint
            if (step_count - last_reset_step > 100) { // Avoid spamming (only reset every ~1.6 seconds)
                std::cout << "\n*** At second-to-last waypoint! Restarting field path... ***\n" << std::endl;
                drivekit::PathGoal new_path(path, 2.0f, 2.0f, false);
                tractor.tracker->set_path(new_path);
                tractor.tracker->smoothen(25.0f);
                last_reset_step = step_count;
            }
        }

        // Print progress every 5 seconds
        if (step_count % 300 == 0) {
            std::cout << "\nTime: " << elapsed << "s\n";
            auto completed = tractor.tracker->is_path_completed();
            std::string status = completed ? "[COMPLETED]" : (robot_stopped[0] ? "[STOPPED]" : "[RUNNING]");
            std::cout << "  Machine 0: (" << std::fixed << std::setprecision(1) << pos.point.x << ", " << pos.point.y
                      << ") " << status << "\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    return 0;
}
