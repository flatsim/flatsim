#include "flatsim/robot.hpp"
#include "concord/geometry/primitives/triangle.hpp"
#include "flatsim/robot/network/interfaces/canbus_interface.hpp"
#include "flatsim/robot/network/interfaces/wifi_interface.hpp"
#include "flatsim/robot/network/interfaces/zenoh_interface.hpp"
#include "flatsim/simulator.hpp"
#include <algorithm>
#include <cmath>

namespace fs {

    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
        : rec(rec), world(world) {
        filter.group = 0;            // Use bit/mask system, not group system
        filter.bit = 1 << group;     // Each robot gets unique bit position
        filter.mask = ~(1 << group); // Exclude own bit from collision mask

        // Initialize tracker
        tracker = std::make_unique<drivekit::Tracker>(drivekit::TrackerType::PID);
    }

    Robot::~Robot() {
        // Cleanup network interfaces
        network.cleanup();
    }

    void Robot::tick(float dt) {
        if (!chassis.exists()) {
            throw NullPointerException("chassis");
        }

        // Update pose from physics FIRST
        this->info.bound.pose.point.x = chassis.get_transform().position.x;
        this->info.bound.pose.point.y = chassis.get_transform().position.y;
        // Physics engine gives angle 90 degrees off - correct it
        this->info.bound.pose.angle.yaw = chassis.get_transform().rotation.GetAngle() + M_PI / 2;

        // Update sensors with current pose (after pose is updated from physics)
        sensors.update_all(info.bound.pose, dt);
        // Note: WGS coordinates can be calculated via point.toWGS(datum) when needed

        // Update navigation controller when enabled
        if (navigation_enabled) {
            update_navigation(dt);
        }

        // Update network interfaces - batch updates to reduce overhead
        network.tick(dt);

        chassis.tick(dt);
        chassis.update(controls.get_steerings(), controls.get_throttles(), dt);

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
        tank.tick(dt, chassis.get_pose());

        // visualize();
    }

    void Robot::init(concord::Datum datum, RobotInfo robo) {
        this->datum = datum;
        this->info = robo;
        this->spawn_position = robo.bound.pose;
        this->original_color = robo.color; // Store original color
        // Synchronize runtime role state with the static robot info so that
        // chain and control logic (MASTER/FOLLOWER/SLAVE) behaves as configured
        this->state.role = robo.role;

        if (!world) {
            throw NullPointerException("world");
        }

        if (!rec) {
            throw NullPointerException("recording stream");
        }

        // Initialize chassis manager
        chassis.init(this, rec, world, filter, robo);

        // Initialize device managers
        controls.init(this, robo);
        chain.init(this);

        // Initialize navigation controller with robot constraints
        drivekit::RobotConstraints constraints;
        // Differential-drive huskies have all zero steering limits
        bool is_differential_drive = true;
        for (float a : robo.controls.steerings_max) {
            if (std::abs(a) > 1e-6f) {
                is_differential_drive = false;
                break;
            }
        }
        constraints.steering_type =
            is_differential_drive ? drivekit::SteeringType::DIFFERENTIAL : drivekit::SteeringType::ACKERMANN;

        // Derive basic geometry from wheel bounds
        if (!robo.wheels.empty()) {
            double max_y = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_x = std::numeric_limits<double>::infinity();

            for (const auto &w : robo.wheels) {
                max_y = std::max(max_y, static_cast<double>(w.pose.point.y));
                min_y = std::min(min_y, static_cast<double>(w.pose.point.y));
                max_x = std::max(max_x, static_cast<double>(w.pose.point.x));
                min_x = std::min(min_x, static_cast<double>(w.pose.point.x));
            }

            double wheelbase = std::abs(max_y - min_y);
            double track_width = std::abs(max_x - min_x);

            constraints.wheelbase = wheelbase > 0.1 ? wheelbase : 1.0;
            constraints.track_width = track_width > 0.1 ? track_width : 1.0;
        } else {
            constraints.wheelbase = 1.5;
            constraints.track_width = 1.5;
        }

        // Use normalized velocity units for navcon ([-1,1] maps to full throttle)
        constraints.max_linear_velocity = 1.0;
        constraints.min_linear_velocity = -1.0;
        constraints.max_linear_acceleration = 1.0;
        constraints.max_angular_velocity = 1.0;

        // Steering limits from robot config
        double max_steer = 0.0;
        for (float a : robo.controls.steerings_max) {
            max_steer = std::max(max_steer, static_cast<double>(std::abs(a)));
        }
        if (max_steer <= 0.0) {
            max_steer = 30.0 * M_PI / 180.0;
        }
        constraints.max_steering_angle = max_steer;
        constraints.max_steering_rate = 1.0; // rad/s, approximate actuator rate

        constraints.min_turning_radius = robo.turning_radius;
        constraints.robot_length = robo.bound.size.y; // Robot length (longitudinal)
        constraints.robot_width = robo.bound.size.x;  // Robot width (lateral)

        tracker->init(constraints, rec, robo.seqid);

        // Configure controller settings
        drivekit::ControllerConfig config;
        config.allow_reverse = false; // TODO: Enable backward maneuvers for tight turns
        tracker->get_controller()->set_config(config);

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
        controls.reset_controls();

        // Break all chain connections before teleporting
        if (propagate) {
            chain.break_chain_for_teleport();
        }

        chassis.teleport(pose);
    }

    void Robot::respawn() {
        controls.reset_controls();

        // Break all chain connections before respawning
        chain.break_chain_for_teleport();

        chassis.teleport(spawn_position);
    }

    void Robot::update_color(const pigment::RGB &new_color) {
        info.color = new_color;
        if (chassis.exists()) {
            chassis.update_color(new_color);
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
        if (chassis.exists()) chassis.tock(label);

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

        // Update tracker visualization
        if (tracker) {
            tracker->tock();
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

    void Robot::get_velocity(double &linear, double &angular) const {
        linear = 0.0;
        angular = 0.0;

        if (!chassis.exists()) {
            return;
        }

        if (const auto *body = chassis.get_body()) {
            const auto &linear_vel = body->GetLinearVelocity();
            const double yaw = info.bound.pose.angle.yaw;
            // Forward velocity along the robot's heading
            linear = linear_vel.x * std::cos(yaw) + linear_vel.y * std::sin(yaw);
            // Angular velocity around Z from the physics body
            angular = body->GetAngularVelocity();
        }
    }

    // Navigation helper method
    void Robot::update_navigation(float dt) {
        if (!tracker) {
            return;
        }

        // Get current robot state for waypoint
        drivekit::RobotState nav_state;
        nav_state.pose = info.bound.pose;
        // Pull current velocities from physics so controllers (e.g. MPC) get an accurate state
        if (auto *body = chassis.get_body()) {
            const auto &linear_vel = body->GetLinearVelocity();
            const double yaw = nav_state.pose.angle.yaw;
            // Forward velocity along the robot's heading
            nav_state.velocity.linear = linear_vel.x * std::cos(yaw) + linear_vel.y * std::sin(yaw);
            // Angular velocity around Z from the physics body
            nav_state.velocity.angular = body->GetAngularVelocity();
        } else {
            nav_state.velocity.linear = 0.0;
            nav_state.velocity.angular = 0.0;
        }
        nav_state.timestamp = 0.0; // TODO: get actual timestamp

        // If this robot is pulling a follower (e.g. trailer), expose its pose
        // to the navigation stack so trailer-aware controllers can use it.
        const auto followers = chain.get_connected_followers();
        if (!followers.empty() && followers.front()) {
            nav_state.has_trailer = true;
            nav_state.trailer_pose = followers.front()->get_position();
        }

        // Pass through behavior flags from robot state
        nav_state.turn_first = state.turn_first;
        nav_state.allow_move = state.allow_move;

        // Compute control command
        auto velocity_cmd = tracker->tick(nav_state, dt);

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

    void Robot::brake() {
        // Apply braking using the chassis brake system
        chassis.brake(fs::constants::brake);
        // Also reset controls to zero
        controls.reset_controls();
    }

    bool Robot::in_line_of_sight(const concord::Point &point, float sight_distance, float half_angle) const {
        // Default sight distance is 4x robot length
        if (sight_distance <= 0.0f) {
            sight_distance = 4.0f * static_cast<float>(info.bound.size.y);
        }

        // Get robot's current position and heading
        const auto &pos = info.bound.pose;
        double heading = pos.angle.yaw;

        // Create vision triangle
        concord::Triangle vision = concord::Triangle::from_vision(pos.point, heading, sight_distance, half_angle);

        return vision.contains(point);
    }

    bool Robot::robot_in_sight(const Robot &other, float sight_distance, float half_angle) const {
        // Don't check against self
        if (&other == this) {
            return false;
        }

        // Default sight distance is 4x robot length
        if (sight_distance <= 0.0f) {
            sight_distance = 4.0f * static_cast<float>(info.bound.size.y);
        }

        // Get robot's current position and heading
        const auto &pos = info.bound.pose;
        double heading = pos.angle.yaw;

        // Create vision triangle
        concord::Triangle vision = concord::Triangle::from_vision(pos.point, heading, sight_distance, half_angle);

        // Get other robot's position and approximate size as a circle
        const auto &other_pos = other.info.bound.pose.point;
        double other_radius = std::max(other.info.bound.size.x, other.info.bound.size.y) / 2.0;

        // Check if other robot's bounding circle intersects our vision triangle
        return vision.intersects_circle(other_pos, other_radius);
    }

    std::vector<Robot *> Robot::get_robots_in_sight(float sight_distance, float half_angle) const {
        std::vector<Robot *> in_sight;

        auto all_robots = get_all_robots();
        for (auto *robot : all_robots) {
            if (robot && robot != this && robot_in_sight(*robot, sight_distance, half_angle)) {
                in_sight.push_back(robot);
            }
        }

        return in_sight;
    }

} // namespace fs
