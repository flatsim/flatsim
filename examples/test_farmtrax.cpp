// Farmtrax Multi-Machine Field Coverage Test
//
// Features: Robot colors, Collision avoidance with LIDAR, Dubins curves for turns
//
// Run:
//   ./build/linux/x86_64/release/test_farmtrax

#include "flatsim/utils.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "echo/widget.hpp"
#include "flatsim/agent.hpp"
#include "flatsim/agent/sensor/lidar_sensor.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include "pigment/pigment.hpp"
#include "rerun/recording_stream.hpp"

#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/turners/dubins.hpp"
#include "flatsim/utils.hpp"

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
float calculate_distance(const datapod::Pose &p1, const datapod::Pose &p2) {
    float dx = p1.point.x - p2.point.x;
    float dy = p1.point.y - p2.point.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Get robot's approximate size (diagonal of bounding box)
float get_robot_size(const agent::Agent &robot) {
    float size_x = robot.machine().config().bound.size.x;
    float size_y = robot.machine().config().bound.size.y;
    return std::sqrt(size_x * size_x + size_y * size_y);
}

// Check if LIDAR detects an obstacle in front within the given range
float check_lidar_forward(fs::LIDARSensor *lidar, float forward_angle_range, const datapod::Pose &robot_pose,
                          std::shared_ptr<rerun::RecordingStream> rec, const std::string &robot_id, pigment::RGB color,
                          bool debug_output = false) {
    if (!lidar) {
        return std::numeric_limits<float>::max();
    }

    const auto &data = lidar->get_lidar_data();

    if (data.ranges.empty()) {
        if (debug_output) {
            echo::debug(robot_id, " LIDAR: no data yet");
        }
        return std::numeric_limits<float>::max();
    }

    float min_distance = data.max_range;
    float min_forward_distance = data.max_range;

    std::vector<std::array<float, 3>> all_beam_starts;
    std::vector<std::array<float, 3>> all_beam_ends;
    std::vector<rerun::Color> all_beam_colors;

    for (size_t i = 0; i < data.ranges.size(); ++i) {
        float angle = data.angles[i];
        float range = data.ranges[i];
        bool is_valid_hit = data.valid[i] && range < data.max_range * 0.99f;
        bool is_forward = std::abs(angle) <= forward_angle_range;

        float world_angle = utils::get_yaw(robot_pose) + angle;

        float start_x = robot_pose.point.x;
        float start_y = robot_pose.point.y;
        float end_x = start_x + range * std::cos(world_angle);
        float end_y = start_y + range * std::sin(world_angle);

        all_beam_starts.push_back({start_x, start_y, 0.5f});
        all_beam_ends.push_back({end_x, end_y, 0.5f});

        if (is_valid_hit) {
            pigment::RGB red_color{255, 0, 0};
            auto mixed_color = color.mix(red_color, 0.5);
            all_beam_colors.push_back(rerun::Color(mixed_color.r(), mixed_color.g(), mixed_color.b()));
            if (is_forward && range < min_forward_distance) {
                min_forward_distance = range;
            }
        } else {
            all_beam_colors.push_back(rerun::Color(color.r(), color.g(), color.b()));
        }

        if (is_valid_hit && range < min_distance) {
            min_distance = range;
        }
    }

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
            echo::debug(robot_id, " LIDAR: ", data.ranges.size(), " beams, min_fwd=", std::fixed, std::setprecision(2),
                        min_forward_distance, "m, min_all=", min_distance, "m, max_range=", data.max_range, "m");
        }
    }

    return min_forward_distance;
}

// Generate smooth path with Dubins curves between swath endpoints
std::vector<datapod::Point> generate_dubins_path(const std::vector<std::shared_ptr<const farmtrax::Swath>> &swaths,
                                                 float turning_radius, float step_size = 0.5f) {
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

            float dx_curr = swath->line.end.x - swath->line.start.x;
            float dy_curr = swath->line.end.y - swath->line.start.y;
            float yaw_end = std::atan2(dy_curr, dx_curr);

            float dx_next = next_swath->line.end.x - next_swath->line.start.x;
            float dy_next = next_swath->line.end.y - next_swath->line.start.y;
            float yaw_start = std::atan2(dy_next, dx_next);

            farmtrax::turners::Pose2D start_pose(swath->line.end, yaw_end);
            farmtrax::turners::Pose2D end_pose(next_swath->line.start, yaw_start);

            auto dubins_path = dubins.plan_path(start_pose, end_pose, step_size);

            for (size_t j = 1; j < dubins_path.waypoints.size() - 1; ++j) {
                path.push_back(dubins_path.waypoints[j].point);
            }
        }
    }

    return path;
}

int main() {
    echo::banner("FARMTRAX TEST", echo::BoxStyle::Double);
    echo::info("Features: Robot colors, Collision avoidance, Dubins curves");
    echo::separator();

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);

    // Create simulator with Rerun
    datapod::Geo world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500, 500, world_datum, rec);

    // Create a simple square field (100m x 100m)
    datapod::Polygon poly;
    poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0});
    poly.vertices.push_back(datapod::Point{100.0, 0.0, 0.0});
    poly.vertices.push_back(datapod::Point{100.0, 100.0, 0.0});
    poly.vertices.push_back(datapod::Point{0.0, 100.0, 0.0});
    poly.vertices.push_back(datapod::Point{0.0, 0.0, 0.0});

    echo::info("Creating square field (100m x 100m)");

    farmtrax::Field field(poly, world_datum, true, 100000.0);
    field.gen_field(4.0, 90.0, 0);

    int num_machines = 3;
    echo::info("Number of machines: ", num_machines);

    auto part_cnt = field.get_parts().size();
    echo::info("Total field parts: ", part_cnt);

    if (field.get_parts().empty()) {
        echo::error("No field parts generated!");
        return 1;
    }

    const auto &part = field.get_parts()[0];
    auto part_area = part.boundary.polygon.area();
    echo::info("Field area: ", std::fixed, std::setprecision(1), part_area, " sq.m (", (part_area / 10000.0),
               " hectares)");
    echo::info("Headlands: ", part.headlands.size(), ", Swaths: ", part.swaths.size());

    auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[0]);
    farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
    divy.compute_division();

    auto &res = divy.result();

    float turning_radius = 5.0f;

    std::vector<int> active_machines;
    std::vector<agent::Agent *> agents;

    // Load tractors and assign paths
    for (int m = 0; m < num_machines; ++m) {
        if (res.swaths_per_machine.at(m).empty()) {
            echo::warn("Machine ", m, " has no swaths assigned");
            continue;
        }

        echo::separator("Machine " + std::to_string(m));
        echo::info("Assigned swaths: ", res.swaths_per_machine.at(m).size());

        farmtrax::Nety nety(res.swaths_per_machine.at(m));
        nety.field_traversal();

        const auto &swaths = nety.get_swaths();
        echo::info("Optimized swaths: ", swaths.size());

        std::vector<datapod::Point> path = generate_dubins_path(swaths, turning_radius, 0.5f);

        if (path.empty()) {
            echo::warn("Machine ", m, " has empty path after optimization");
            continue;
        }

        echo::info("Generated path with ", path.size(), " waypoints (including Dubins curves)");

        float spawn_x = path[0].x;
        float spawn_y = path[0].y - (m * 8.0f);
        float spawn_yaw = 0.0f;

        if (path.size() > 1) {
            float dx = path[1].x - path[0].x;
            float dy = path[1].y - path[0].y;
            spawn_yaw = std::atan2(dy, dx);
        }

        // Spawn tractor with unique UUID and color
        std::string uuid = "tractor_" + std::to_string(m);
        datapod::Pose spawn_pose = utils::make_pose_2d(spawn_x, spawn_y, spawn_yaw - 1.5708f);
        auto &tractor =
            sim.spawn_agent("examples/machines/tractor.json", spawn_pose, uuid, ROBOT_COLORS[m % ROBOT_COLORS.size()]);

        echo::info("Loaded tractor ", m, " at (", spawn_x, ", ", spawn_y, ") UUID: ", uuid);

        // Get robot size for LIDAR configuration
        float robot_size = get_robot_size(tractor);

        // Configure LIDAR on the simulator's machine (simulator will do raycasting)
        types::LidarConfig lidar_cfg;
        lidar_cfg.enabled = true;
        lidar_cfg.min_range = robot_size + 0.5f; // min range > robot size
        lidar_cfg.max_range = 15.0f;             // 15m max range
        lidar_cfg.fov_deg = 90.0f;               // 45 degree FOV
        lidar_cfg.resolution_deg = 2.0f;         // 3 degree resolution
        sim.set_lidar_config(uuid, lidar_cfg);

        // Add LIDAR sensor on agent side to receive data from simulator
        auto lidar = std::make_unique<fs::LIDARSensor>(fs::LIDARSensor::ScanPattern::SECTOR_2D,
                                                       10.0f,                   // 10 Hz update rate
                                                       lidar_cfg.min_range,     // min range
                                                       lidar_cfg.max_range,     // max range
                                                       lidar_cfg.fov_deg,       // FOV
                                                       lidar_cfg.resolution_deg // resolution
        );
        tractor.machine().sensors.add(std::move(lidar));
        echo::info("Added LIDAR sensor to Robot ", m);

        // Configure MPPI controller
        tractor.controls().tracker().set_controller_type(drivekit::TrackerType::MPPI);
        tractor.controls().tracker().set_enabled(true);
        tractor.set_navigation_enabled(true);

        auto *mppi_controller = dynamic_cast<drivekit::pred::MPPIFollower *>(tractor.tracker()->get_controller());

        if (mppi_controller) {
            auto mppi_config = mppi_controller->get_mppi_config();
            mppi_config.horizon_steps = 20;
            mppi_config.dt = 0.1;
            mppi_config.num_samples = 1000;
            mppi_config.temperature = 0.1;
            mppi_config.steering_noise = 0.15;
            mppi_config.acceleration_noise = 0.1;
            mppi_config.ref_velocity = 0.6;
            mppi_config.weight_cte = 200.0;
            mppi_config.weight_epsi = 180.0;
            mppi_config.weight_vel = 1.0;
            mppi_config.weight_steering = 80.0;
            mppi_config.weight_acceleration = 20.0;
            mppi_controller->set_mppi_config(mppi_config);
        }

        drivekit::PathGoal path_goal(path, 2.0f, 2.0f, false);
        tractor.tracker()->set_path(path_goal);
        tractor.tracker()->smoothen(25.0f);

        active_machines.push_back(m);
        agents.push_back(&tractor);
        echo::info("Path set with ", path.size(), " waypoints");
    }

    echo::separator("SIMULATION START");
    echo::info("Active machines: ", active_machines.size());
    echo::info("Collision avoidance: Robots will stop when LIDAR detects obstacle");

    std::vector<bool> robot_stopped(num_machines, false);

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f;
    int step_count = 0;
    bool all_completed = false;

    while (!all_completed) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed > 600) {
            echo::warn("Timeout reached!");
            break;
        }

        // Collision avoidance using LIDAR
        for (size_t i = 0; i < agents.size(); ++i) {
            int m = active_machines[i];
            auto &robot = *agents[i];
            float size_m = get_robot_size(robot);

            auto *lidar = robot.machine().sensors.get<fs::LIDARSensor>();
            float safe_distance = 2.0f * size_m;

            bool should_stop = false;

            if (lidar) {
                auto pos_m = robot.get_position();
                bool debug = false; // Disable LIDAR debug output (too noisy)

                float min_obstacle_dist =
                    check_lidar_forward(lidar, 0.52f, pos_m, rec, robot.uuid(), robot.machine().config().color, debug);

                if (min_obstacle_dist < safe_distance) {
                    should_stop = true;
                    if (!robot_stopped[m]) {
                        echo::warn("Robot ", m, " stopping (LIDAR detected obstacle at ", std::fixed,
                                   std::setprecision(1), min_obstacle_dist, "m, safe=", safe_distance, "m)");
                    }
                }
            }

            if (should_stop && !robot_stopped[m]) {
                robot.machine().set_navigation_enabled(false);
                robot.brake();
                robot_stopped[m] = true;
            } else if (!should_stop && robot_stopped[m]) {
                robot.machine().set_navigation_enabled(true);
                robot_stopped[m] = false;
                echo::info("Robot ", m, " resuming");
            }

            if (robot_stopped[m]) {
                robot.brake();
            }
        }

        sim.tick(dt);
        sim.tock();

        // Check if all machines completed their paths
        all_completed = true;
        for (size_t i = 0; i < agents.size(); ++i) {
            if (!agents[i]->tracker()->is_path_completed()) {
                all_completed = false;
            }
        }

        // Print progress every 5 seconds
        if (step_count % 300 == 0) {
            echo::separator("Time: " + std::to_string(elapsed) + "s");
            for (size_t i = 0; i < agents.size(); ++i) {
                int m = active_machines[i];
                auto pos = agents[i]->get_position();
                auto completed = agents[i]->tracker()->is_path_completed();
                std::string status = completed ? "[COMPLETED]" : (robot_stopped[m] ? "[STOPPED]" : "[RUNNING]");
                echo::info("Machine ", m, ": (", std::fixed, std::setprecision(1), pos.point.x, ", ", pos.point.y, ") ",
                           status);
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    echo::separator();
    if (all_completed) {
        echo::box("All machines completed their paths!", echo::BoxStyle::Double);
    } else {
        echo::box("Simulation ended (timeout or incomplete)", echo::BoxStyle::Dashed);
    }

    echo::info("Total field work time: ", step_count / 60.0f, " seconds");
    echo::info("Check Rerun visualization for complete field coverage");

    return 0;
}
