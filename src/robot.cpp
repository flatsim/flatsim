#include "flatsim/robot.hpp"
#include "flatsim/network/interfaces/canbus_interface.hpp"
#include "flatsim/network/interfaces/wifi_interface.hpp"
#include "flatsim/network/interfaces/zenoh_interface.hpp"
#include "flatsim/simulator.hpp"
#include <algorithm>
#include <cmath>

namespace fs {

    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
        : rec(rec), world(world) {
        filter.group = 0;            // Use bit/mask system, not group system
        filter.bit = 1 << group;     // Each robot gets unique bit position
        filter.mask = ~(1 << group); // Exclude own bit from collision mask

        // Initialize navigation controller
        navcon = std::make_unique<navcon::Navcon>(navcon::NavconControllerType::PID);
    }

    Robot::~Robot() {
        // Cleanup network interfaces
        network.cleanup();
    }

    void Robot::tick(float dt) {
        // Optimize sensor updates - use manager
        sensors.update_all(info.bound.pose, dt);

        if (!chassis) {
            throw NullPointerException("chassis");
        }

        this->info.bound.pose.point.x = chassis->get_transform().position.x;
        this->info.bound.pose.point.y = chassis->get_transform().position.y;
        // Physics engine gives angle 90 degrees off - correct it
        this->info.bound.pose.angle.yaw = chassis->get_transform().rotation.GetAngle() + M_PI / 2;
        // Note: WGS coordinates can be calculated via point.toWGS(datum) when needed

        // Update navigation controller
        update_navigation(dt);

        // Update network interfaces - batch updates to reduce overhead
        network.tick(dt);

        chassis->tick(dt);
        chassis->update(controls.get_steerings(), controls.get_throttles(), dt);

        // Update power consumption based on operation mode
        if (power.is_powered()) {
            float consumption_multiplier = 1.0f;
            switch (state.mode) {
            case OP::IDLE:
                consumption_multiplier = 0.1f; // Minimal consumption when idle
                break;
            case OP::TRANSPORT:
                consumption_multiplier = 0.5f; // Medium consumption when moving
                break;
            case OP::WORK:
                consumption_multiplier = 1.5f; // Higher consumption when working
                break;
            case OP::CHARGING:
                consumption_multiplier = 0.0f; // No consumption when charging
                power.charge(dt);              // Charge battery
                break;
            default:
                consumption_multiplier = 0.1f;
            }
            power.update(dt, consumption_multiplier);
        }

        // Update tank if present
        tank.tick(dt, chassis->get_pose());

        // visualize();
    }

    void Robot::init(concord::Datum datum, RobotInfo robo) {
        spdlog::info("Initializing robot {}...", robo.name);
        this->datum = datum;
        this->info = robo;
        this->spawn_position = robo.bound.pose;
        this->original_color = robo.color; // Store original color

        if (!world) {
            throw NullPointerException("world");
        }

        if (!rec) {
            throw NullPointerException("recording stream");
        }

        chassis = std::make_unique<Chassis>(world, rec, filter, &info, &state);
        if (!chassis) {
            throw InitializationException("chassis creation failed");
        }
        chassis->init(robo);

        // Initialize device managers
        controls.init(this, robo);
        chain.init(this);

        // Initialize navigation controller with robot constraints
        navcon::RobotConstraints constraints;
        constraints.wheelbase = 3.0;   // Reasonable wheelbase for tractor
        constraints.track_width = 2.0; // Reasonable track width

        // Use actual robot throttle limits
        float max_throttle = 0.0f;
        for (size_t i = 0; i < robo.controls.throttles_max.size(); ++i) {
            max_throttle = std::max(max_throttle, robo.controls.throttles_max[i]);
        }
        constraints.max_linear_velocity = max_throttle;
        constraints.min_linear_velocity = -max_throttle;

        // Set reasonable navigation limits
        constraints.max_steering_angle = 35.0f * M_PI / 180.0f; // 35 degrees in radians
        constraints.max_angular_velocity = 1.0f;                // 1 rad/s
        constraints.min_turning_radius = robo.turning_radius;
        constraints.robot_length = robo.bound.size.y; // Robot length (longitudinal)
        constraints.robot_width = robo.bound.size.x;  // Robot width (lateral)

        navcon->init(constraints, rec, robo.seqid);

        // Initialize tank if present
        if (robo.tank.has_value()) {
            tank.init(robo.tank.value(), info.color, info.seqid);
        }

        // Initialize power source if present
        if (robo.power_source.has_value()) {
            power.init(robo.power_source.value());
        }

        // Initialize follower capabilities based on robot configuration
        chain.update_follower_capabilities();

        // Configure network interfaces based on robot role
        switch (info.role) {
        case RobotRole::MASTER:
            // MASTER robots have Zenoh + WiFi interfaces for maximum connectivity
            network.add_interface(std::make_unique<fs::network::ZenohInterface>());
            network.add_interface(std::make_unique<fs::network::WiFiInterface>());
            break;
        case RobotRole::FOLLOWER:
            // FOLLOWER robots have Zenoh + CAN-bus interfaces
            network.add_interface(std::make_unique<fs::network::ZenohInterface>());
            network.add_interface(std::make_unique<fs::network::CANBusInterface>());
            break;
        case RobotRole::SLAVE:
            // SLAVE robots have CAN-bus only (no network interfaces for autonomy)
            network.add_interface(std::make_unique<fs::network::CANBusInterface>());
            break;
        }

        // Initialize network with robot UUID
        network.init(info.uuid);
    }

    void Robot::teleport(concord::Pose pose) { teleport(pose, true); }

    void Robot::teleport(concord::Pose pose, bool propagate) {
        spdlog::info("Teleporting robot {} to ({:.2f}, {:.2f}) - breaking chain connections", info.name, pose.point.x,
                     pose.point.y);
        controls.reset_controls();

        // Break all chain connections before teleporting
        if (propagate) {
            chain.break_chain_for_teleport();
        }

        chassis->teleport(pose);
    }

    void Robot::respawn() {
        spdlog::info("Respawning robot {} - breaking chain connections", info.name);
        controls.reset_controls();

        // Break all chain connections before respawning
        chain.break_chain_for_teleport();

        chassis->teleport(spawn_position);
    }

    void Robot::update_color(const pigment::RGB &new_color) {
        info.color = new_color;
        if (chassis) {
            chassis->update_color(new_color);
        }
    }

    void Robot::update(float angular, float linear) {
        controls.set_angular(angular);
        controls.set_linear(linear);
    }

    // Spatial queries - robot can find other robots
    std::vector<Robot *> Robot::get_all_robots() const {
        if (!simulator) {
            return {};
        }
        return simulator->get_all_robots();
    }

    Robot *Robot::get_closest_robot(float max_distance) const {
        if (!simulator) {
            return nullptr;
        }
        auto all_robots = get_all_robots();
        Robot *closest = nullptr;
        float min_dist = max_distance;

        auto my_pos = get_position().point;
        for (Robot *other : all_robots) {
            if (other == this) continue;

            auto other_pos = other->get_position().point;
            float dx = my_pos.x - other_pos.x;
            float dy = my_pos.y - other_pos.y;
            float dist_sq = dx * dx + dy * dy;
            float min_dist_sq = min_dist * min_dist;

            if (dist_sq < min_dist_sq) {
                min_dist = std::sqrt(dist_sq);
                closest = other;
            }
        }

        return closest;
    }

    void Robot::tock() {
        if (!state.online) {
            clean();
            return;
        }
        // Create label with role prefix and power percentage
        std::string role_prefix;
        switch (state.role) {
        case RobotRole::MASTER:
            role_prefix = "(M)";
            break;
        case RobotRole::FOLLOWER:
            role_prefix = "(F)";
            break;
        case RobotRole::SLAVE:
            role_prefix = "(S)";
            break;
        }
        std::string label = role_prefix + info.seqid;
        if (power.exists()) label += "(" + std::to_string(static_cast<int>(power.get_percentage())) + "%)";
        if (chassis) chassis->tock(label);

        // Visualize tank if present
        tank.tock(rec);

        auto x = this->info.bound.pose.point.x;
        auto y = this->info.bound.pose.point.y;

        // Use direct initialization - no heap allocations for single-element arrays
        rerun::Color color(info.color.r, info.color.g, info.color.b);

        // 3D position visualization
        rec->log_static(this->info.seqid + "/pose", rerun::Points3D({{float(x), float(y), 0.1f}}).with_colors({color}));

        // GPS coordinates visualization
        auto wgs_coords = this->info.bound.pose.point.toWGS(datum);
        rec->log_static(this->info.seqid + "/pose",
                        rerun::GeoPoints({{float(wgs_coords.lat), float(wgs_coords.lon)}}).with_colors({color}));

        // Update navigation visualization
        if (navcon) {
            navcon->tock();
        }
    }

    void Robot::clean() {
        rec->log(info.seqid, rerun::Clear::RECURSIVE);
        rec->log_with_static(info.seqid, true, rerun::Clear::RECURSIVE);
    }

    void Robot::visualize_pulse(float p_s, float gps_mult, float inc) {
        if (!pulsing) {
            return;
        }

        // Simple pulse implementation - just log basic pulse state
        if (rec) {
            auto pos = get_position();
            rec->log(info.seqid + "/pulse", rerun::Points2D({rerun::Position2D(pos.point.x, pos.point.y)})
                                                .with_colors({rerun::Color(255, 255, 255, 200)})
                                                .with_radii({2.0f}));
        }

        // Reset pulsing after some time
        pulsing = false;
    }

    // Navigation helper method
    void Robot::update_navigation(float dt) {
        if (!navcon) {
            return;
        }

        // Get current robot state
        navcon::RobotState state;
        state.pose = info.bound.pose;
        state.velocity.linear = 0.0;  // TODO: get from robot if available
        state.velocity.angular = 0.0; // TODO: get from robot if available
        state.timestamp = 0.0;        // TODO: get actual timestamp

        // Compute control command
        auto velocity_cmd = navcon->tick(state, dt);

        if (velocity_cmd.valid) {
            // Debug output every 50 calls
            static int nav_debug_count = 0;
            if (nav_debug_count % 50 == 0) {
                std::cout << "NAV CMD: linear=" << velocity_cmd.linear_velocity
                          << ", angular=" << velocity_cmd.angular_velocity
                          << " (inverted: " << -velocity_cmd.angular_velocity << ")" << std::endl;
            }
            nav_debug_count++;

            // Apply velocity command directly
            // Note: Robot uses opposite angular velocity convention (positive = CW)
            // while navcon uses standard convention (positive = CCW)
            controls.set_linear(velocity_cmd.linear_velocity);
            controls.set_angular(-velocity_cmd.angular_velocity); // Invert for robot's convention
        }
    }

} // namespace fs
