#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "concord/concord.hpp"
#include "flatsim/core/loader.hpp"
#include "flatsim/robot/sensor/lidar_sensor.hpp"
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

// Calculate distance between two robots
float calculate_distance(const concord::Pose &p1, const concord::Pose &p2) {
    float dx = p1.point.x - p2.point.x;
    float dy = p1.point.y - p2.point.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Get robot's approximate size (diagonal of bounding box)
float get_robot_size(const fs::Robot &robot) {
    // Size uses x, y, z coordinates (x = length, y = width typically)
    float size_x = robot.info.bound.size.x;
    float size_y = robot.info.bound.size.y;
    return std::sqrt(size_x * size_x + size_y * size_y);
}

// Check if LIDAR detects an obstacle in front within the given range
// Returns the minimum distance to an obstacle in the forward sector, or max_range if clear
// Also visualizes ALL beams in Rerun (forward beams highlighted)
float check_lidar_forward(fs::LIDARSensor *lidar, float forward_angle_range, const concord::Pose &robot_pose,
                          std::shared_ptr<rerun::RecordingStream> rec, const std::string &robot_id, pigment::RGB color,
                          bool debug_output = false) {
    if (!lidar) {
        return std::numeric_limits<float>::max();
    }

    const auto &data = lidar->get_lidar_data();

    // If no data yet, return max (no obstacle)
    if (data.ranges.empty()) {
        if (debug_output) {
            std::cout << robot_id << " LIDAR: no data yet\n";
        }
        return std::numeric_limits<float>::max();
    }

    float min_distance = data.max_range;
    float min_forward_distance = data.max_range;

    // Collect ALL beams for visualization
    std::vector<std::array<float, 3>> all_beam_starts;
    std::vector<std::array<float, 3>> all_beam_ends;
    std::vector<rerun::Color> all_beam_colors;

    // LIDAR SECTOR_2D scans from -FOV/2 to +FOV/2
    for (size_t i = 0; i < data.ranges.size(); ++i) {
        float angle = data.angles[i]; // This is relative to sensor/robot heading
        float range = data.ranges[i];
        bool is_valid_hit = data.valid[i] && range < data.max_range * 0.99f;
        bool is_forward = std::abs(angle) <= forward_angle_range;

        // Transform beam to world coordinates for visualization
        float world_angle = robot_pose.angle.yaw + angle;

        // Visualize the beam
        float start_x = robot_pose.point.x;
        float start_y = robot_pose.point.y;
        float end_x = start_x + range * std::cos(world_angle);
        float end_y = start_y + range * std::sin(world_angle);

        all_beam_starts.push_back({start_x, start_y, 0.5f});
        all_beam_ends.push_back({end_x, end_y, 0.5f});

        // Color coding:
        // - Beams with hit (obstacle): Mix tractor color with red (toned down red toward tractor color)
        // - Beams without hit (clear): Tractor's color with transparency
        if (is_valid_hit) {
            // Mix tractor color with red (70% red, 30% tractor color for a toned-down red)
            pigment::RGB red_color{255, 0, 0};
            auto mixed_color = color.mix(red_color, 0.5);
            all_beam_colors.push_back(rerun::Color(mixed_color.r, mixed_color.g, mixed_color.b, 255));
            if (is_forward && range < min_forward_distance) {
                min_forward_distance = range;
            }
        } else {
            // Use tractor's color for clear beams
            all_beam_colors.push_back(rerun::Color(color.r, color.g, color.b, 150));
        }

        // Track overall minimum
        if (is_valid_hit && range < min_distance) {
            min_distance = range;
        }
    }

    // Visualize ALL beams in Rerun as line strips
    if (rec && !all_beam_starts.empty()) {
        std::vector<rerun::LineStrip3D> lines;
        for (size_t i = 0; i < all_beam_starts.size(); ++i) {
            lines.push_back(
                rerun::LineStrip3D({rerun::Vec3D(all_beam_starts[i][0], all_beam_starts[i][1], all_beam_starts[i][2]),
                                    rerun::Vec3D(all_beam_ends[i][0], all_beam_ends[i][1], all_beam_ends[i][2])}));
        }

        rec->log_static(robot_id + "/lidar",
                        rerun::LineStrips3D(lines).with_colors(all_beam_colors).with_radii({0.01f}));

        if (debug_output) {
            std::cout << robot_id << " LIDAR: " << data.ranges.size() << " beams, min_fwd=" << std::fixed
                      << std::setprecision(2) << min_forward_distance << "m, min_all=" << min_distance
                      << "m, max_range=" << data.max_range << "m\n";
        }
    }

    return min_forward_distance;
}

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

    // Create a simple square field (100m x 100m)
    concord::Polygon poly;
    poly.addPoint(concord::Point{0.0, 0.0, 0.0});
    poly.addPoint(concord::Point{100.0, 0.0, 0.0});
    poly.addPoint(concord::Point{100.0, 100.0, 0.0});
    poly.addPoint(concord::Point{0.0, 100.0, 0.0});
    poly.addPoint(concord::Point{0.0, 0.0, 0.0}); // Close the polygon

    std::cout << "Creating square field (100m x 100m)\n";

    farmtrax::Field field(poly, world_datum, true, 100000.0);

    // Generate field with 4m swath width, 0 degree angle, 2 headland passes
    field.gen_field(4.0, 0.0, 2);

    int num_machines = 3; // Number of tractors
    std::cout << "Number of machines: " << num_machines << "\n";

    auto part_cnt = field.get_parts().size();
    std::cout << "Total field parts: " << part_cnt << "\n";

    // Process the first field part (in a simple square, there should be only one part)
    if (field.get_parts().empty()) {
        std::cerr << "No field parts generated!\n";
        return 1;
    }

    const auto &part = field.get_parts()[0];
    auto part_area = boost::geometry::area(part.boundary.b_polygon);
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
        std::vector<concord::Point> path = generate_dubins_path(swaths, turning_radius, 0.5f);

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

        // Update robot color after loading
        tractor.update_color(ROBOT_COLORS[m % ROBOT_COLORS.size()]);

        // Add LIDAR sensor for collision detection
        // Using SECTOR_2D pattern with 60 degree FOV, 15m range, 5 degree resolution
        // min_range must be larger than robot size to avoid self-detection
        float robot_size = get_robot_size(tractor);
        auto lidar = std::make_unique<fs::LIDARSensor>(simulator.get_world().get_world(),
                                                       fs::LIDARSensor::ScanPattern::SECTOR_2D,
                                                       10.0,              // 10 Hz update rate
                                                       robot_size + 0.5f, // min range > robot size
                                                       15.0,              // 15m max range
                                                       45.0,              // 60 degree FOV (30 deg each side)
                                                       3.0                // 4 degree resolution = 15 rays
        );
        lidar->configure_noise(0.0, 0.0, 0.0);             // No noise for debugging
        lidar->set_collision_filter(tractor.get_filter()); // Use robot's filter to ignore own body
        tractor.sensors.add(std::move(lidar));
        std::cout << "Added LIDAR sensor to Robot " << m << " (filter bit=" << tractor.get_filter().bit << ")\n";

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

    // Simulation loop
    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS
    int step_count = 0;
    bool all_completed = false;

    while (!all_completed) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 600) { // 10 minute timeout
            std::cout << "Timeout reached!\n";
            break;
        }

        // Collision avoidance: use LIDAR to detect obstacles in front
        for (int m = 0; m < num_machines; ++m) {
            if (m >= simulator.num_robots()) continue;

            auto &robot_m = simulator.get_robot(m);
            float size_m = get_robot_size(robot_m);

            // Get LIDAR sensor and check for obstacles
            auto *lidar = robot_m.sensors.get<fs::LIDARSensor>();
            float safe_distance = 2.0f * size_m; // Stop if obstacle within 2x robot size

            bool should_stop = false;

            if (lidar) {
                // Check LIDAR for obstacles in forward sector (within ~30 degrees)
                auto pos_m = robot_m.get_position();
                // Debug output for first 10 steps
                bool debug = (step_count < 10);
                // Use the robot's actual seqid for Rerun entity path
                float min_obstacle_dist =
                    check_lidar_forward(lidar, 0.52f, pos_m, rec, robot_m.info.seqid, robot_m.info.color, debug);

                if (min_obstacle_dist < safe_distance) {
                    should_stop = true;
                    if (!robot_stopped[m]) {
                        std::cout << "Robot " << m << " stopping (LIDAR detected obstacle at " << std::fixed
                                  << std::setprecision(1) << min_obstacle_dist << "m, safe=" << safe_distance << "m)\n";
                    }
                }
            }

            // Update robot movement state using allow_move flag and apply braking
            if (should_stop && !robot_stopped[m]) {
                robot_m.state.allow_move = false;
                robot_m.brake(); // Apply physical braking force
                robot_stopped[m] = true;
            } else if (!should_stop && robot_stopped[m]) {
                robot_m.state.allow_move = true;
                robot_stopped[m] = false;
                std::cout << "Robot " << m << " resuming\n";
            }

            // Keep applying brake while stopped to ensure robot stays stationary
            if (robot_stopped[m]) {
                robot_m.brake();
            }
        }

        simulator.tick(dt);
        simulator.tock(5);

        // Check if all machines completed their paths
        all_completed = true;
        for (int m = 0; m < num_machines; ++m) {
            if (m >= simulator.num_robots()) continue;
            auto &tractor = simulator.get_robot(m);
            if (!tractor.tracker->is_path_completed()) {
                all_completed = false;
            }
        }

        // Print progress every 5 seconds
        if (step_count % 300 == 0) {
            std::cout << "\nTime: " << elapsed << "s\n";
            for (int m = 0; m < num_machines; ++m) {
                if (m >= simulator.num_robots()) continue;
                auto &tractor = simulator.get_robot(m);
                auto pos = tractor.get_position();
                auto completed = tractor.tracker->is_path_completed();
                std::string status = completed ? "[COMPLETED]" : (robot_stopped[m] ? "[STOPPED]" : "[RUNNING]");
                std::cout << "  Machine " << m << ": (" << std::fixed << std::setprecision(1) << pos.point.x << ", "
                          << pos.point.y << ") " << status << "\n";
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    if (all_completed) {
        std::cout << "\n=== All machines completed their paths! ===" << std::endl;
    } else {
        std::cout << "\n=== Simulation ended (timeout or incomplete) ===" << std::endl;
    }

    std::cout << "Total field work time: " << step_count / 60.0f << " seconds\n";
    std::cout << "Check Rerun visualization for complete field coverage\n";

    return 0;
}
