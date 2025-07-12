#pragma once

#include "concord/concord.hpp"
#include "flatsim/types.hpp"
#include "navcon.hpp"
#include <memory>
#include <vector>

namespace fs {
    // Forward declaration
    class Robot;

    // Goal types for navigation
    struct NavigationGoal {
        concord::Point target;
        float tolerance = 1.0f;  // meters
        float max_speed = 1.0f;  // m/s
        
        NavigationGoal(concord::Point t, float tol = 1.0f, float speed = 1.0f) 
            : target(t), tolerance(tol), max_speed(speed) {}
    };

    struct PathGoal {
        std::vector<concord::Point> waypoints;
        float tolerance = 1.0f;  // meters
        float max_speed = 1.0f;  // m/s
        bool loop = false;       // whether to loop back to start
        
        PathGoal(std::vector<concord::Point> wp, float tol = 1.0f, float speed = 1.0f, bool l = false)
            : waypoints(wp), tolerance(tol), max_speed(speed), loop(l) {}
    };

    // Controller types
    enum class ControllerType {
        PID,
        PURE_PURSUIT,
        STANLEY,
        CARROT
    };

    // ============================================================================
    // NavigationController - High-level navigation controller using navcon
    // ============================================================================
    class NavigationController {
    private:
        Robot* robot;
        ControllerType controller_type;
        navcon::RobotConfig navcon_config;
        std::unique_ptr<navcon::Controller<navcon::RobotState, navcon::VelocityCommand>> velocity_controller;
        std::unique_ptr<navcon::WheelController> wheel_controller;
        std::shared_ptr<rerun::RecordingStream> rec;  // For path visualization
        
        // Current navigation state
        std::optional<NavigationGoal> current_goal;
        std::optional<PathGoal> current_path;
        size_t current_waypoint_index = 0;
        bool goal_reached = false;
        bool path_completed = false;
        
        // Controller parameters
        struct ControllerParams {
            // PID parameters - much more conservative to prevent oscillation
            float linear_kp = 0.2f, linear_ki = 0.0f, linear_kd = 0.02f;
            float angular_kp = 0.3f, angular_ki = 0.0f, angular_kd = 0.02f;
            
            // Pure pursuit parameters
            float lookahead_distance = 2.5f;  // Shorter distance to stay closer to path
            float lookahead_gain = 0.5f;
            
            // Stanley parameters
            float cross_track_gain = 1.0f;
            float softening_gain = 1.0f;
            
            // Carrot parameters
            float carrot_distance = 1.0f;
        } params;

    public:
        NavigationController(Robot* r, ControllerType type = ControllerType::PID);
        ~NavigationController() = default;

        // Initialize with robot configuration
        void init(const RobotInfo& robot_info);
        
        // Goal management
        void set_goal(const NavigationGoal& goal);
        void set_path(const PathGoal& path);
        void clear_goal();
        void clear_path();
        
        // Navigation control
        void update(float dt);
        bool is_goal_reached() const { return goal_reached; }
        bool is_path_completed() const { return path_completed; }
        
        // Controller configuration
        void set_controller_type(ControllerType type);
        void set_controller_params(const ControllerParams& new_params) { params = new_params; }
        const ControllerParams& get_controller_params() const { return params; }
        
        // Status information
        float get_distance_to_goal() const;
        float get_distance_to_current_waypoint() const;
        concord::Point get_current_target() const;
        
        // Emergency stop
        void emergency_stop();
        
    private:
        // Internal helper methods
        navcon::RobotConfig create_navcon_config(const RobotInfo& robot_info);
        navcon::RobotState get_robot_state() const;
        navcon::Goal get_current_navcon_goal() const;
        void apply_wheel_commands(const navcon::WheelCommands& commands);
        void apply_velocity_command(const navcon::VelocityCommand& cmd);
        void update_waypoint_progress();
        void create_controller();
        
        // Coordinate system conversions
        navcon::Point to_navcon_point(const concord::Point& point) const;
        concord::Point to_concord_point(const navcon::Point& point) const;
        
        // Visualization methods
        void visualize_current_path() const;
        void visualize_navigation_goal() const;
        void visualize_robot_direction() const;
    };

} // namespace fs