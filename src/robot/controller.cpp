#include "flatsim/robot/controller.hpp"
#include "flatsim/robot.hpp"
#include "flatsim/utils.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace fs {

    NavigationController::NavigationController(Robot *r, ControllerType type) : robot(r), controller_type(type) {}

    void NavigationController::init(const RobotInfo &robot_info) {
        // Create the navigation controller
        create_controller();

        // Get recording stream from robot for visualization
        rec = robot->rec;
    }


    void NavigationController::create_controller() {
        navcon::ControllerConfig config;

        switch (controller_type) {
        case ControllerType::PID:
            std::cout << "Creating PID controller..." << std::endl;
            config.kp_linear = params.linear_kp;
            config.ki_linear = params.linear_ki;
            config.kd_linear = params.linear_kd;
            config.kp_angular = params.angular_kp;
            config.ki_angular = params.angular_ki;
            config.kd_angular = params.angular_kd;
            controller = navcon::create_controller("pid", config);
            std::cout << "PID controller created: " << (controller ? "success" : "failed") << std::endl;
            break;

        case ControllerType::PURE_PURSUIT:
            std::cout << "Creating Pure Pursuit controller..." << std::endl;
            config.lookahead_distance = params.lookahead_distance;
            controller = navcon::create_controller("pure_pursuit", config);
            std::cout << "Pure Pursuit controller created: " << (controller ? "success" : "failed") << std::endl;
            break;

        case ControllerType::STANLEY:
            std::cout << "Creating Stanley controller..." << std::endl;
            config.k_cross_track = params.cross_track_gain;
            config.k_heading = params.softening_gain;
            controller = navcon::create_controller("stanley", config);
            std::cout << "Stanley controller created: " << (controller ? "success" : "failed") << std::endl;
            break;

        case ControllerType::CARROT:
            config.lookahead_distance = params.carrot_distance;
            controller = navcon::create_controller("carrot", config);
            break;
        }
    }

    void NavigationController::set_goal(const NavigationGoal &goal) {
        std::cout << "NavigationController: Setting goal to (" << goal.target.x << ", " << goal.target.y
                  << ") with tolerance " << goal.tolerance << std::endl;
        current_goal = goal;
        current_path.reset();
        goal_reached = false;
        path_completed = false;
    }

    void NavigationController::set_path(const PathGoal &path) {
        current_path = path;
        current_goal.reset();
        current_waypoint_index = 0;
        goal_reached = false;
        path_completed = false;
        
        // Convert PathGoal to navcon::Path and set it in the controller
        if (controller) {
            navcon::Path navcon_path;
            for (const auto& waypoint : path.waypoints) {
                navcon::Pose wp;
                wp.point = waypoint;
                wp.angle = concord::Euler{0.0f, 0.0f, 0.0f}; // No specific heading required
                navcon_path.waypoints.push_back(wp);
            }
            navcon_path.is_closed = path.loop; // Set loop behavior
            controller->set_path(navcon_path);
        }
    }

    void NavigationController::clear_goal() {
        current_goal.reset();
        goal_reached = false;
    }

    void NavigationController::clear_path() {
        current_path.reset();
        current_waypoint_index = 0;
        path_completed = false;
        
        // Clear path in the navcon controller
        if (controller) {
            navcon::Path empty_path;
            controller->set_path(empty_path);
        }
    }

    void NavigationController::update(float dt) {
        if (!controller) {
            static bool warned = false;
            if (!warned) {
                std::cout << "NavigationController: controller=" << (controller ? "valid" : "null") << std::endl;
                warned = true;
            }
            return;
        }

        // Update waypoint progress for path following
        if (current_path.has_value()) {
            update_waypoint_progress();
        }

        // Check if we have a valid target
        if (!current_goal.has_value() && !current_path.has_value()) {
            return;
        }

        // Get current robot state
        navcon::RobotState state = get_robot_state();

        // Get current target goal
        // NOTE: For Pure Pursuit, pass empty goal since it uses the path directly
        navcon::Goal goal;
        if (controller_type == ControllerType::PID || controller_type == ControllerType::STANLEY || controller_type == ControllerType::CARROT) {
            goal = get_current_navcon_goal(); // Point-based controllers need specific targets
        }
        // Pure Pursuit uses the path set with set_path(), not individual goals

        // Create constraints from actual robot configuration
        navcon::RobotConstraints constraints;
        constraints.wheelbase = 3.0;   // Reasonable wheelbase for tractor
        constraints.track_width = 2.0; // Reasonable track width

        // Use actual robot throttle limits from JSON
        float max_throttle = 0.0f;
        for (size_t i = 0; i < robot->info.controls.throttles_max.size(); ++i) {
            max_throttle = std::max(max_throttle, robot->info.controls.throttles_max[i]);
        }
        constraints.max_linear_velocity = max_throttle; // Use actual throttle value: 0.2 is 0.2!
        constraints.min_linear_velocity = -max_throttle;

        // Use reasonable navigation limits for angular velocity
        constraints.max_steering_angle = 35.0f * M_PI / 180.0f; // 35 degrees in radians (typical tractor)
        constraints.max_angular_velocity = 1.0f; // 1 rad/s = ~57 degrees/sec - reasonable for navigation

        // Compute control command
        auto velocity_cmd = controller->compute_control(state, goal, constraints, dt);

        // Debug output disabled for production
        // static int debug_count = 0;
        // if (debug_count % 30 == 0) {
        //     std::cout << "Goal(" << goal.target_pose.point.x << "," << goal.target_pose.point.y
        //              << "), Robot(" << state.pose.point.x << "," << state.pose.point.y << "), Heading=" <<
        //              state.pose.angle.yaw << std::endl;
        // }
        // debug_count++;

        // Apply velocity command directly
        apply_velocity_command(velocity_cmd);

        // Check goal completion
        if (current_goal.has_value()) {
            float distance = get_distance_to_goal();
            goal_reached = distance <= current_goal->tolerance;
        }

        if (current_path.has_value() && current_waypoint_index >= current_path->waypoints.size()) {
            path_completed = true;
        }

        // Update visualization
        visualize_current_path();
        visualize_navigation_goal();
        visualize_robot_direction();
    }

    navcon::RobotState NavigationController::get_robot_state() const {
        navcon::RobotState state;
        const concord::Pose &pose = robot->get_position();

        // Debug output disabled
        // static int debug_state_count = 0;
        // if (debug_state_count % 50 == 0) {
        //     std::cout << "COORD DEBUG: Flatsim pose - pos(" << pose.point.x << "," << pose.point.y
        //              << "), yaw=" << pose.angle.yaw << std::endl;
        // }
        // debug_state_count++;

        state.pose = pose;
        
        state.velocity.linear = 0.0;  // TODO: get from robot if available
        state.velocity.angular = 0.0; // TODO: get from robot if available
        state.timestamp = 0.0;        // TODO: get actual timestamp

        return state;
    }

    navcon::Goal NavigationController::get_current_navcon_goal() const {
        navcon::Goal goal;

        if (current_goal.has_value()) {
            goal.target_pose = concord::Pose{current_goal->target, concord::Euler{0.0f, 0.0f, 0.0f}};
            goal.tolerance_position = current_goal->tolerance;
        } else if (current_path.has_value() && current_waypoint_index < current_path->waypoints.size()) {
            goal.target_pose =
                concord::Pose{current_path->waypoints[current_waypoint_index], concord::Euler{0.0f, 0.0f, 0.0f}};
            goal.tolerance_position = current_path->tolerance;
        }

        return goal;
    }


    void NavigationController::apply_velocity_command(const navcon::VelocityCommand &cmd) {
        // Simply pass through the velocity commands to the robot
        // The robot is responsible for handling its own scaling and limits
        
        // Debug output for angular control issues
        static int apply_debug_count = 0;
        if (apply_debug_count % 50 == 0) { // Debug output
            auto pos = robot->get_position();
            auto target = get_current_target();
            
            // Calculate distance to target
            double distance_to_target = std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));
            
            // Calculate desired heading and error
            double desired_heading = std::atan2(target.y - pos.point.y, target.x - pos.point.x);
            double heading_error = desired_heading - pos.angle.yaw;
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
            
            // Convert angles to degrees for easier reading
            double current_yaw_deg = pos.angle.yaw * 180.0 / M_PI;
            double desired_heading_deg = desired_heading * 180.0 / M_PI;
            double heading_error_deg = heading_error * 180.0 / M_PI;

            std::cout << "=== DETAILED NAV DEBUG ===" << std::endl;
            std::cout << "Robot Position: (" << pos.point.x << ", " << pos.point.y << ")" << std::endl;
            std::cout << "Target Position: (" << target.x << ", " << target.y << ")" << std::endl;
            std::cout << "Distance to Target: " << distance_to_target << " meters" << std::endl;
            std::cout << "Current Yaw: " << pos.angle.yaw << " rad (" << current_yaw_deg << " deg)" << std::endl;
            std::cout << "Desired Heading: " << desired_heading << " rad (" << desired_heading_deg << " deg)" << std::endl;
            std::cout << "Heading Error: " << heading_error << " rad (" << heading_error_deg << " deg)" << std::endl;
            std::cout << "NavCon Commands: linear=" << cmd.linear_velocity << ", angular=" << cmd.angular_velocity << std::endl;
            std::cout << "Waypoint Index: " << current_waypoint_index << " / " << (current_path.has_value() ? current_path->waypoints.size() : 0) << std::endl;
            std::cout << "=========================" << std::endl;
        }
        apply_debug_count++;
        
        // Direct pass-through to robot's control methods
        // Note: Robot uses opposite angular velocity convention (positive = CW)
        // while navcon uses standard convention (positive = CCW)
        robot->set_linear(cmd.linear_velocity);
        robot->set_angular(-cmd.angular_velocity);  // Invert for robot's convention
    }

    void NavigationController::update_waypoint_progress() {
        if (!current_path.has_value() || current_waypoint_index >= current_path->waypoints.size()) {
            return;
        }

        float distance = get_distance_to_current_waypoint();
        
        // Debug waypoint progression
        static int waypoint_debug_count = 0;
        if (waypoint_debug_count % 50 == 0) {
            auto robot_pos = robot->get_position().point;
            auto current_target = current_path->waypoints[current_waypoint_index];
            std::cout << "WAYPOINT DEBUG: Index=" << current_waypoint_index 
                      << ", Robot(" << robot_pos.x << "," << robot_pos.y << ")"
                      << ", Target(" << current_target.x << "," << current_target.y << ")"
                      << ", Distance=" << distance << ", Tolerance=" << current_path->tolerance << std::endl;
        }
        waypoint_debug_count++;

        if (distance <= current_path->tolerance) {
            std::cout << "WAYPOINT REACHED! Moving to next waypoint. Index was " << current_waypoint_index;
            current_waypoint_index++;
            std::cout << ", now " << current_waypoint_index << std::endl;

            // Check if we've completed the path
            if (current_waypoint_index >= current_path->waypoints.size()) {
                if (current_path->loop && !current_path->waypoints.empty()) {
                    current_waypoint_index = 0; // Loop back to start
                } else {
                    path_completed = true;
                }
            }
        }
    }

    float NavigationController::get_distance_to_goal() const {
        if (!current_goal.has_value()) {
            return std::numeric_limits<float>::infinity();
        }

        const concord::Point &robot_pos = robot->get_position().point;
        const concord::Point &target = current_goal->target;

        return std::sqrt(std::pow(target.x - robot_pos.x, 2) + std::pow(target.y - robot_pos.y, 2));
    }

    float NavigationController::get_distance_to_current_waypoint() const {
        if (!current_path.has_value() || current_waypoint_index >= current_path->waypoints.size()) {
            return std::numeric_limits<float>::infinity();
        }

        const concord::Point &robot_pos = robot->get_position().point;
        const concord::Point &target = current_path->waypoints[current_waypoint_index];

        return std::sqrt(std::pow(target.x - robot_pos.x, 2) + std::pow(target.y - robot_pos.y, 2));
    }

    concord::Point NavigationController::get_current_target() const {
        if (current_goal.has_value()) {
            return current_goal->target;
        } else if (current_path.has_value() && current_waypoint_index < current_path->waypoints.size()) {
            return current_path->waypoints[current_waypoint_index];
        }
        return concord::Point{0, 0};
    }

    void NavigationController::set_controller_type(ControllerType type) {
        controller_type = type;
        create_controller();
    }

    void NavigationController::emergency_stop() {
        robot->set_linear(0.0f);
        robot->set_angular(0.0f);
        clear_goal();
        clear_path();
    }


    void NavigationController::visualize_current_path() const {
        if (!current_path.has_value() || !rec) return;

        // Convert waypoints to 3D coordinates for visualization
        std::vector<std::array<float, 3>> path_points;
        for (const auto &waypoint : current_path->waypoints) {
            path_points.push_back({static_cast<float>(waypoint.x), static_cast<float>(waypoint.y), 0.3f});
        }

        // Draw the planned path as a green line
        if (path_points.size() >= 2) {
            auto path_line = rerun::components::LineStrip3D(path_points);
            rec->log_static(robot->info.name + "/navigation/planned_path",
                            rerun::LineStrips3D(path_line)
                                .with_colors({{0, 255, 0}}) // Green for planned path
                                .with_radii({{0.15f}}));
        }

        // Visualize individual waypoints as green spheres
        std::vector<rerun::components::Position3D> waypoint_positions;
        for (const auto &waypoint : current_path->waypoints) {
            waypoint_positions.push_back({static_cast<float>(waypoint.x), static_cast<float>(waypoint.y), 0.3f});
        }

        if (!waypoint_positions.empty()) {
            rec->log_static(robot->info.name + "/navigation/waypoints",
                            rerun::Points3D(waypoint_positions)
                                .with_colors({{0, 255, 0}}) // Green waypoints
                                .with_radii({{0.4f}}));
        }

        // Highlight current target waypoint in yellow
        if (current_waypoint_index < current_path->waypoints.size()) {
            const auto &current_target = current_path->waypoints[current_waypoint_index];
            rec->log_static(
                robot->info.name + "/navigation/current_target",
                rerun::Points3D({{static_cast<float>(current_target.x), static_cast<float>(current_target.y), 0.4f}})
                    .with_colors({{255, 255, 0}}) // Yellow for current target
                    .with_radii({{0.6f}}));
        }
    }

    void NavigationController::visualize_navigation_goal() const {
        if (!current_goal.has_value() || !rec) return;

        const auto &target = current_goal->target;

        // Visualize goal point as a red sphere
        rec->log_static(robot->info.name + "/navigation/goal",
                        rerun::Points3D({{static_cast<float>(target.x), static_cast<float>(target.y), 0.3f}})
                            .with_colors({{255, 0, 0}}) // Red for goal
                            .with_radii({{0.5f}}));

        // Visualize tolerance circle around goal
        std::vector<std::array<float, 3>> tolerance_circle;
        int segments = 32;
        for (int i = 0; i <= segments; ++i) {
            float angle = 2.0f * M_PI * i / segments;
            float x = target.x + current_goal->tolerance * std::cos(angle);
            float y = target.y + current_goal->tolerance * std::sin(angle);
            tolerance_circle.push_back({x, y, 0.1f});
        }

        if (!tolerance_circle.empty()) {
            auto tolerance_line = rerun::components::LineStrip3D(tolerance_circle);
            rec->log_static(robot->info.name + "/navigation/goal_tolerance",
                            rerun::LineStrips3D(tolerance_line)
                                .with_colors({{255, 0, 0, 128}}) // Semi-transparent red
                                .with_radii({{0.08f}}));
        }
    }

    void NavigationController::visualize_robot_direction() const {
        if (!rec) return;

        auto robot_pos = robot->get_position();
        auto target = get_current_target();

        // Draw line from robot to current target (orange)
        std::vector<std::array<float, 3>> direction_line = {
            {static_cast<float>(robot_pos.point.x), static_cast<float>(robot_pos.point.y), 0.2f},
            {static_cast<float>(target.x), static_cast<float>(target.y), 0.2f}};

        auto direction_strip = rerun::components::LineStrip3D(direction_line);
        rec->log_static(robot->info.name + "/navigation/direction",
                        rerun::LineStrips3D(direction_strip)
                            .with_colors({{255, 165, 0}}) // Orange for direction
                            .with_radii({{0.1f}}));

        // Visualize robot heading as a purple arrow
        float heading_length = 2.0f;
        float heading_x = robot_pos.point.x + heading_length * std::cos(robot_pos.angle.yaw);
        float heading_y = robot_pos.point.y + heading_length * std::sin(robot_pos.angle.yaw);

        std::vector<std::array<float, 3>> heading_line = {
            {static_cast<float>(robot_pos.point.x), static_cast<float>(robot_pos.point.y), 0.25f},
            {heading_x, heading_y, 0.25f}};

        auto heading_strip = rerun::components::LineStrip3D(heading_line);
        rec->log_static(robot->info.name + "/navigation/heading",
                        rerun::LineStrips3D(heading_strip)
                            .with_colors({{128, 0, 128}}) // Purple for heading
                            .with_radii({{0.12f}}));
    }

} // namespace fs
