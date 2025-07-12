# Flatsim Navigation Controller Integration

## Overview

Flatsim integrates the NavCon navigation controller library through a high-level wrapper class `NavigationController` that provides a clean interface between Flatsim's robot system and NavCon's algorithms. This integration handles coordinate transformation, velocity scaling, and visualization while keeping the navigation logic separate from the simulation framework.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            FLATSIM LAYER                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│  Robot Class                    NavigationController Class                  │
│  ┌─────────────────┐           ┌─────────────────────────────────────────┐   │
│  │ • Position      │           │ • Goal Management                       │   │
│  │ • Velocity      │    ←──→   │ • Path Following                        │   │
│  │ • set_linear()  │           │ • Parameter Tuning                      │   │
│  │ • set_angular() │           │ • Waypoint Progress                     │   │
│  └─────────────────┘           │ • Visualization                         │   │
│                                 └─────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────────┤
│                            INTEGRATION LAYER                               │
├─────────────────────────────────────────────────────────────────────────────┤
│  Coordinate Transform          Velocity Scaling          Visualization      │
│  ┌─────────────────┐           ┌─────────────────┐      ┌─────────────────┐  │
│  │ Flatsim Pose    │           │ NavCon Velocity │      │ Rerun 3D        │  │
│  │      ↕          │           │      ↕          │      │ • Paths         │  │
│  │ NavCon State    │           │ Robot Commands  │      │ • Goals         │  │
│  └─────────────────┘           └─────────────────┘      │ • Progress      │  │
│                                                          └─────────────────┘  │
├─────────────────────────────────────────────────────────────────────────────┤
│                             NAVCON LAYER                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │     PID     │  │Pure Pursuit │  │   Stanley   │  │   Carrot    │         │
│  │ Controller  │  │ Controller  │  │ Controller  │  │ Controller  │         │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Components

### 1. NavigationController Class

**Location**: `include/flatsim/robot/controller.hpp`, `src/robot/controller.cpp`

The `NavigationController` class serves as the main integration point between Flatsim and NavCon:

```cpp
class NavigationController {
private:
    Robot* robot;                                    // Reference to Flatsim robot
    ControllerType controller_type;                  // Current algorithm type
    std::unique_ptr<navcon::Controller> controller;  // NavCon algorithm instance
    std::shared_ptr<rerun::RecordingStream> rec;     // Visualization system
    
    // Navigation state
    std::optional<NavigationGoal> current_goal;      // Point-to-point goals
    std::optional<PathGoal> current_path;            // Path following goals
    size_t current_waypoint_index = 0;               // Current path progress
    bool goal_reached = false;                       // Goal completion flag
    bool path_completed = false;                     // Path completion flag
    
    // Algorithm parameters
    struct ControllerParams {
        // PID gains
        float linear_kp = 0.2f, linear_ki = 0.0f, linear_kd = 0.02f;
        float angular_kp = 0.3f, angular_ki = 0.0f, angular_kd = 0.02f;
        
        // Pure Pursuit parameters
        float lookahead_distance = 2.5f;
        float lookahead_gain = 0.5f;
        
        // Stanley parameters
        float cross_track_gain = 2.5f;
        float softening_gain = 1.5f;
        
        // Carrot parameters
        float carrot_distance = 1.0f;
    } params;
};
```

### 2. Goal Types

Flatsim defines its own goal types that are converted to NavCon format:

```cpp
// Point-to-point navigation
struct NavigationGoal {
    concord::Point target;          // Target position
    float tolerance = 1.0f;         // Acceptable error (meters)
    float max_speed = 1.0f;         // Maximum speed (m/s)
};

// Path following navigation
struct PathGoal {
    std::vector<concord::Point> waypoints;  // Sequence of waypoints
    float tolerance = 1.0f;                 // Waypoint tolerance (meters)
    float max_speed = 1.0f;                 // Maximum speed (m/s)
    bool loop = false;                      // Loop back to start
};
```

### 3. Controller Types

Flatsim supports all four NavCon algorithms:

```cpp
enum class ControllerType {
    PID,           // Point-to-point with PID control
    PURE_PURSUIT,  // Path following with geometric lookahead
    STANLEY,       // Path following with cross-track error correction
    CARROT         // Simple goal chasing
};
```

## Integration Flow

### 1. Initialization

```cpp
// Create navigation controller for a robot
auto nav_controller = std::make_unique<NavigationController>(robot, ControllerType::STANLEY);

// Initialize with robot configuration
nav_controller->init(robot->info);

// Optionally tune parameters
NavigationController::ControllerParams params;
params.cross_track_gain = 3.0f;  // Tune for specific robot
nav_controller->set_controller_params(params);
```

### 2. Data Flow During Update Cycle

```cpp
void NavigationController::update(float dt) {
    // 1. Get robot state from Flatsim
    navcon::RobotState state = get_robot_state();
    
    // 2. Get current navigation goal
    navcon::Goal goal = get_current_navcon_goal();
    
    // 3. Setup robot constraints from configuration
    navcon::RobotConstraints constraints;
    constraints.wheelbase = 3.0;                           // From robot config
    constraints.max_linear_velocity = robot->info.controls.throttles_max[0];
    constraints.max_angular_velocity = 1.0;
    
    // 4. Compute navigation command using NavCon
    auto velocity_cmd = controller->compute_control(state, goal, constraints, dt);
    
    // 5. Apply velocity command to robot
    apply_velocity_command(velocity_cmd);
    
    // 6. Update navigation state and visualization
    update_waypoint_progress();
    visualize_current_path();
    visualize_navigation_goal();
}
```

### 3. Coordinate Transformation

The integration handles coordinate system conversion between Flatsim and NavCon:

```cpp
navcon::RobotState NavigationController::get_robot_state() const {
    navcon::RobotState state;
    const concord::Pose& pose = robot->get_position();
    
    // Direct pose mapping (both use Concord coordinate system)
    state.pose = pose;
    
    // Velocity mapping (to be implemented from robot dynamics)
    state.velocity.linear = 0.0;   // TODO: get from robot
    state.velocity.angular = 0.0;  // TODO: get from robot
    state.timestamp = 0.0;         // TODO: get actual timestamp
    
    return state;
}
```

### 4. Velocity Command Application

The integration converts NavCon's velocity commands to Flatsim's robot interface:

```cpp
void NavigationController::apply_velocity_command(const navcon::VelocityCommand& cmd) {
    // Direct pass-through to robot's velocity interface
    // Note: Robot uses opposite angular velocity convention
    robot->set_linear(cmd.linear_velocity);
    robot->set_angular(-cmd.angular_velocity);  // Invert for robot's convention
}
```

## Usage Examples

### 1. Point-to-Point Navigation

```cpp
// Set a navigation goal
NavigationGoal goal(concord::Point{10.0, 5.0}, 0.5f, 1.0f);
nav_controller->set_goal(goal);

// Update in simulation loop
while (!nav_controller->is_goal_reached()) {
    nav_controller->update(dt);
    // Robot automatically navigates to goal
}
```

### 2. Path Following

```cpp
// Define a path
std::vector<concord::Point> waypoints = {
    {0.0, 0.0}, {5.0, 0.0}, {10.0, 5.0}, {10.0, 10.0}
};
PathGoal path(waypoints, 0.5f, 1.5f, false);
nav_controller->set_path(path);

// Update in simulation loop
while (!nav_controller->is_path_completed()) {
    nav_controller->update(dt);
    // Robot follows the waypoint sequence
}
```

### 3. Algorithm Switching

```cpp
// Start with PID for simple navigation
nav_controller->set_controller_type(ControllerType::PID);

// Switch to Pure Pursuit for smooth path following
nav_controller->set_controller_type(ControllerType::PURE_PURSUIT);

// Use Stanley for precise path tracking
nav_controller->set_controller_type(ControllerType::STANLEY);
```

## Visualization Integration

The integration provides rich 3D visualization through Rerun:

```cpp
void NavigationController::visualize_current_path() const {
    if (!current_path.has_value() || !rec) return;
    
    // Draw planned path as green line
    std::vector<std::array<float, 3>> path_points;
    for (const auto& waypoint : current_path->waypoints) {
        path_points.push_back({waypoint.x, waypoint.y, 0.3f});
    }
    
    rec->log_static(robot->info.name + "/navigation/planned_path",
                    rerun::LineStrips3D(rerun::components::LineStrip3D(path_points))
                        .with_colors({{0, 255, 0}})    // Green
                        .with_radii({{0.15f}}));
}
```

## Configuration Integration

Robot configuration from Flatsim's JSON is used to setup NavCon constraints:

```cpp
void NavigationController::init(const RobotInfo& robot_info) {
    create_controller();
    rec = robot->rec;  // Share visualization system
    
    // Robot constraints are derived from Flatsim's robot configuration
    // - Wheelbase from chassis dimensions
    // - Max velocities from throttle/steering limits
    // - Physical dimensions for collision avoidance
}
```

## Key Integration Features

### 1. **Seamless Coordinate Mapping**
- Both Flatsim and NavCon use Concord geometry library
- Direct pose mapping without coordinate transformation
- Consistent angle conventions (with sign correction for angular velocity)

### 2. **Parameter Persistence**
- Controller parameters stored in NavigationController
- Easy tuning through getter/setter interface
- Parameters persist across algorithm switches

### 3. **Waypoint Management**
- Automatic waypoint progression tracking
- Loop/non-loop path support
- Distance-based waypoint advancement

### 4. **Real-time Visualization**
- Live path visualization in 3D
- Goal markers and tolerance circles
- Robot heading and direction indicators
- Current target highlighting

### 5. **Goal Management**
- Support for both point goals and path goals
- Goal completion detection
- Emergency stop functionality

### 6. **Debug Information**
- Detailed navigation status logging
- Distance and heading error reporting
- Controller status monitoring

## Best Practices

### 1. **Controller Selection**
- Use **PID** for simple point-to-point navigation
- Use **Pure Pursuit** for smooth curved paths
- Use **Stanley** for precise path tracking
- Use **Carrot** for basic goal chasing with minimal tuning

### 2. **Parameter Tuning**
- Start with default parameters
- Tune incrementally based on robot behavior
- Consider robot size and dynamics when setting gains
- Test in simulation before deploying to real hardware

### 3. **Path Quality**
- Ensure waypoints are appropriately spaced
- Avoid sharp turns that exceed robot capabilities
- Consider robot turning radius when planning paths
- Use smooth curves for better tracking performance

### 4. **Performance Monitoring**
- Monitor goal completion status
- Track distance and heading errors
- Watch for oscillation or instability
- Use visualization for debugging path following issues

This integration provides a clean, maintainable interface between Flatsim's simulation environment and NavCon's navigation algorithms, enabling sophisticated autonomous navigation behaviors while keeping the systems loosely coupled.