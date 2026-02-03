// Field GPS/IMU + NMEA output demo (LOCAL mode - single process)
//
// Demonstrates GNSS sensor with NMEA serial output via PTY.
// The agent47 Agent automatically converts ENU position to WGS84 coordinates.
//
// Run:
//   ./build/test_field_gps_nmea
//
// Then connect to the PTY to see NMEA output:
//   cat /tmp/flatsim_gnss

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <agent47.hpp>
#include <echo/widget.hpp>
#include <nonsens/nonsens.hpp>
#include <pigment/pigment.hpp>
#include <wirebit/wirebit.hpp>

#include <farmtrax/divy.hpp>
#include <farmtrax/field.hpp>
#include <farmtrax/graph.hpp>
#include <farmtrax/turners/dubins.hpp>

static const std::vector<pigment::RGB> ROBOT_COLORS = {
    pigment::RGB{90, 196, 185}, // Turquoise
    pigment::RGB{90, 153, 196}, // Sky Blue
    pigment::RGB{90, 101, 196}, // Periwinkle
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

    echo::banner("FIELD GPS/IMU + NMEA DEMO (LOCAL)", echo::BoxStyle::Double);
    echo::info("GNSS output via PTY - connect with: cat /tmp/flatsim_gnss");
    echo::separator();

    // ------------------------------------------------------------------------
    // Setup PTY for NMEA serial output
    // ------------------------------------------------------------------------
    const char *link_path = "/tmp/flatsim_gnss";

    auto pty_result = wirebit::PtyLink::create({.raw_bytes = true});
    if (pty_result.is_err()) {
        echo::error("Failed to create PTY link");
        return 1;
    }
    auto pty = std::move(pty_result.value());
    auto slave_path = std::string(pty.slave_path().c_str());
    auto link = std::make_shared<wirebit::PtyLink>(std::move(pty));

    // Create symlink for easy access
    std::error_code ec;
    std::filesystem::remove(link_path, ec);
    std::filesystem::create_symlink(slave_path, link_path, ec);
    if (ec) {
        echo::warn("Failed to create symlink ", link_path, " -> ", slave_path, ": ", ec.message());
    }

    wirebit::SerialConfig serial_cfg{};
    wirebit::SerialEndpoint serial(link, serial_cfg, 1);

    echo::info("GNSS PTY slave: ", slave_path);
    echo::info("Symlink:        ", link_path);
    echo::separator();

    // ------------------------------------------------------------------------
    // Setup simulator and field
    // ------------------------------------------------------------------------
    const int num_machines = 1;
    echo::info("Number of machines: ", num_machines);

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    // Create a large square field
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
    echo::separator("Using part 0");
    echo::info("Field area: ", std::fixed, std::setprecision(1), part_area, " sq.m (", (part_area / 10000.0),
               " hectares)");
    echo::info("Headlands: ", part.headlands.size(), ", Swaths: ", part.swaths.size());

    auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[0]);
    farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
    divy.compute_division();
    auto &res = divy.result();

    float turning_radius = 5.0f;
    std::vector<datapod::Point> path;

    if (!res.swaths_per_machine.empty() && !res.swaths_per_machine.at(0).empty()) {
        farmtrax::Nety nety(res.swaths_per_machine.at(0));
        nety.field_traversal();
        const auto &swaths = nety.get_swaths();
        path = generate_dubins_path(swaths, turning_radius, 0.5f);
    }

    if (path.size() < 2) {
        echo::error("Generated path is empty/too small");
        return 1;
    }

    // ------------------------------------------------------------------------
    // Spawn tractor
    // ------------------------------------------------------------------------
    float spawn_x = path[0].x;
    float spawn_y = path[0].y;
    float spawn_yaw = std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);

    const std::string uuid = "field_gps_tractor_0";
    const auto color = ROBOT_COLORS[0];

    auto &tractor = sim.spawn_agent("machines/urdf/tractor.urdf",
                                    utils::make_pose_2d(spawn_x, spawn_y, spawn_yaw - 1.5708f), uuid, color);

    echo::info("Robot UUID: ", tractor.uuid());

    // ------------------------------------------------------------------------
    // Setup GNSS sensor via agent47
    // ------------------------------------------------------------------------
    // Create a standalone agent47::Agent for sensor management.
    // In LOCAL mode, flatsim's Agent doesn't create an agent47 instance,
    // so we create one here just for the nonsens sensor pipeline.
    dp::robot::Robot robot_model;
    robot_model.id.uuid = dp::sugar::uuid::generate_v4();
    robot_model.id.name = dp::String(uuid.c_str());
    robot_model.id.ip = dp::sugar::ip::from_string("0.0.0.0");

    agent47::Agent agent(robot_model, nullptr, datum);

    // Add GNSS sensor
    auto sensor_res = agent.add_sensor("gnss", nonsens::sensor::SensorType::GNSS);
    if (!sensor_res.is_ok()) {
        echo::error("Failed to create GNSS sensor: ", sensor_res.error().message.c_str());
        return 1;
    }

    auto *sensor = agent.get_sensor("gnss");
    if (!sensor) {
        echo::error("Failed to get GNSS sensor");
        return 1;
    }

    // Add serial endpoint for NMEA output
    auto output_res = sensor->add_output(nonsens::sensor::Endpoint{&serial});
    if (!output_res.is_ok()) {
        echo::error("Failed to add serial output: ", output_res.error().message.c_str());
        return 1;
    }

    // Get GNSS pod for updating values
    auto podv = sensor->pod();
    auto *gnss = dp::get<nonsens::pod::Gnss *>(podv);
    if (!gnss) {
        echo::error("Failed to get GNSS pod");
        return 1;
    }

    // Initialize GNSS status
    gnss->status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
    gnss->status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
    gnss->rtk_status = nonsens::pod::Gnss::RtkStatus::RTK_FIXED;
    gnss->num_satellites = 12;
    gnss->hdop = 0.8;
    gnss->horizontal_accuracy = 0.02;
    gnss->vertical_accuracy = 0.03;
    gnss->phtg = false;

    echo::info("GNSS sensor configured with NMEA output");

    // ------------------------------------------------------------------------
    // Setup path following
    // ------------------------------------------------------------------------
    tractor.tracker()->set_controller_type(drivekit::TrackerType::MPPI);
    tractor.set_tracker_enabled(true);

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

    echo::separator();
    echo::box("GPS outputting NMEA - Press Ctrl+C to stop", echo::BoxStyle::Dashed);
    echo::info("Monitor with: cat ", link_path);

    // ------------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------------
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

        // Update agent47 with current pose for coordinate conversion
        agent47::types::Feedback fb;
        fb.pose = tractor.get_position();
        fb.twist.linear.vx = tractor.get_linear_velocity();
        fb.twist.angular.vz = tractor.get_angular_velocity();
        agent.update(fb);

        // Update GNSS pod with converted coordinates
        gnss->latitude = agent.geopos.latitude;
        gnss->longitude = agent.geopos.longitude;
        gnss->altitude = agent.geopos.altitude;

        // Push sensor data (outputs NMEA via serial)
        auto push_res = agent.nonsens_.push_all();
        if (!push_res.is_ok()) {
            echo::warn("Failed to push sensor data: ", push_res.error().message.c_str());
        }

        // Toggle PHTG status every 10 seconds (600 steps at ~60 FPS)
        if (step_count % 600 == 0 && step_count > 0) {
            phtg = !phtg;
            gnss->phtg = phtg;
            echo::info("PHTG status: ", phtg ? "ON" : "OFF");
        }

        // Print position every 2 seconds
        if (step_count % 120 == 0) {
            echo::info("Pos: (", std::fixed, std::setprecision(2), tractor.get_position().point.x, ", ",
                       tractor.get_position().point.y, ") -> GPS: (", std::setprecision(8), agent.geopos.latitude, ", ",
                       agent.geopos.longitude, ")");
        }

        // Restart path when near completion
        const auto pos = tractor.get_position();
        const auto second_to_last = path[path.size() - 2];
        const float dx = static_cast<float>(pos.point.x) - second_to_last.x;
        const float dy = static_cast<float>(pos.point.y) - second_to_last.y;
        const float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 3.0f && (step_count - last_reset_step) > 100) {
            tractor.tracker()->set_path(drivekit::PathGoal(path, 2.0f, 2.0f, false));
            tractor.tracker()->smoothen(25.0f);
            last_reset_step = step_count;
            echo::info("Path reset - starting new lap");
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}
