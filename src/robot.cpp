#include "flatsim/robot.hpp"
#include "flatsim/simulator.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <thread>
#include <chrono>

namespace fs {

// Forward declarations
class Robot;

// ============================================================================
// ControlSystem - Handles movement control and propagation through chains
// ============================================================================
class ControlSystem {
private:
    Robot* robot;
    std::vector<float> steerings, throttles;
    std::vector<float> steerings_max, throttles_max;
    std::vector<float> steerings_diff;

public:
    ControlSystem(Robot* r) : robot(r) {}
    
    void init(const RobotInfo& robo);
    void reset_controls();
    void set_angular(float angular);
    void set_linear(float linear);
    void set_angular_as_follower(float angular, const Robot& master);
    void set_linear_as_follower(float linear, const Robot& master);
    
    const std::vector<float>& get_steerings() const { return steerings; }
    const std::vector<float>& get_throttles() const { return throttles; }
    const std::vector<float>& get_steerings_max() const { return steerings_max; }
    const std::vector<float>& get_throttles_max() const { return throttles_max; }
};

// ============================================================================
// ChainManager - Handles all connection/disconnection and chain management
// ============================================================================
class ChainManager {
private:
    Robot* robot;
    std::vector<Robot*> connected_followers;
    std::vector<muli::RevoluteJoint*> connection_joints;
    Robot* master_robot = nullptr;
    FollowerCapabilities follower_capabilities;

public:
    ChainManager(Robot* r) : robot(r) {}
    
    // Connection management
    bool try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>>& all_robots);
    bool try_connect_nearby();
    bool try_connect_from_chain_end();
    
    // Disconnection management
    void disconnect_trailer();
    void disconnect_all_followers();
    void disconnect_last_follower();
    void disconnect_at_position(int position);
    void disconnect_from_position(int position);
    
    // Chain queries
    bool is_connected() const;
    std::vector<Robot*> get_connected_followers() const { return connected_followers; }
    Robot* get_master_robot() const { return master_robot; }
    bool is_follower() const { return master_robot != nullptr; }
    Robot* get_root_master() const;
    std::vector<Robot*> get_full_chain() const;
    int get_chain_length() const;
    int get_position_in_chain() const;
    void print_chain_status() const;
    
    // Capability management
    void update_follower_capabilities();
    const FollowerCapabilities& get_follower_capabilities() const { return follower_capabilities; }
    bool has_steering_capability() const { return follower_capabilities.has_steering; }
    bool has_throttle_capability() const { return follower_capabilities.has_throttle; }
    bool has_available_master_hitches() const { return follower_capabilities.has_additional_hitches; }
    
    // Allow Robot to set master relationship
    void set_master_robot(Robot* master) { master_robot = master; }
    
    // Teleport handling - break connections
    void break_chain_for_teleport();
    void add_follower(Robot* follower, muli::RevoluteJoint* joint) {
        connected_followers.push_back(follower);
        connection_joints.push_back(joint);
        
        spdlog::info("Connected {} as follower to {}", follower->info.name, robot->info.name);
    }
};

    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
        : rec(rec), world(world) {
        filter.group = 0;            // Use bit/mask system, not group system
        filter.bit = 1 << group;     // Each robot gets unique bit position
        filter.mask = ~(1 << group); // Exclude own bit from collision mask
        
        // Initialize modular systems
        control_system = std::make_unique<ControlSystem>(this);
        chain_manager = std::make_unique<ChainManager>(this);
    }
    Robot::~Robot() {}

    void Robot::tick(float dt) {
        for (auto &sensor : sensors) {
            if (!sensor) {
                continue; // Skip null sensors
            }
            sensor->set_robot_pose(info.bound.pose);
            sensor->update(dt);
        }

        if (!chassis) {
            throw NullPointerException("chassis");
        }

        this->info.bound.pose.point.x = chassis->get_transform().position.x;
        this->info.bound.pose.point.y = chassis->get_transform().position.y;
        // Note: WGS coordinates can be calculated via point.toWGS(datum) when needed
        chassis->tick(dt);
        chassis->update(control_system->get_steerings(), control_system->get_throttles(), dt);

        // Update power consumption based on operation mode
        if (power && *power && is_powered()) {
            float consumption_multiplier = 1.0f;
            switch (mode) {
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
                (*power)->charge(dt);          // Charge battery
                break;
            default:
                consumption_multiplier = 0.1f;
            }
            (*power)->update(dt, consumption_multiplier);
        }

        // Update tank if present
        if (tank.has_value()) {
            tank->tick(dt, chassis->get_pose(), rec);
        }

        visualize();
    }

    void Robot::init(concord::Datum datum, RobotInfo robo) {
        spdlog::info("Initializing robot {}...", robo.name);
        this->datum = datum;
        this->info = robo;
        this->role = robo.role; // Set role from RobotInfo
        this->spawn_position = robo.bound.pose;
        this->original_color = robo.color; // Store original color

        if (!world) {
            throw NullPointerException("world");
        }

        if (!rec) {
            throw NullPointerException("recording stream");
        }

        chassis = std::make_unique<Chassis>(world, rec, filter);
        if (!chassis) {
            throw InitializationException("chassis creation failed");
        }
        chassis->init(robo);

        // Initialize control system
        control_system->init(robo);

        // Initialize tank if present
        if (robo.tank.has_value()) {
            tank = Tank(robo.tank->name, Tank::Type::HARVEST, robo.tank->capacity, 0.0f, 0.0f);
            tank->init(info.color, info.name, robo.tank->bound);
        }

        // Initialize power source if present
        if (robo.power_source.has_value()) {
            auto power_type =
                (robo.power_source->type == PowerType::BATTERY) ? Power::Type::BATTERY : Power::Type::FUEL;
            power = std::make_unique<Power>(robo.power_source->name, power_type, robo.power_source->capacity,
                                            robo.power_source->consumption_rate, robo.power_source->charge_rate);
        }

        // Initialize follower capabilities based on robot configuration
        chain_manager->update_follower_capabilities();
    }

    // All control and chain methods are now delegated to ControlSystem and ChainManager
    // Core Robot methods remain here

    void Robot::reset_controls() {
        control_system->reset_controls();
    }

    void Robot::set_angular(float angular) {
        control_system->set_angular(angular);
    }

    void Robot::set_linear(float linear) {
        control_system->set_linear(linear);
    }

    void Robot::set_angular_as_follower(float angular, const Robot& master) {
        control_system->set_angular_as_follower(angular, master);
    }

    void Robot::set_linear_as_follower(float linear, const Robot& master) {
        control_system->set_linear_as_follower(linear, master);
    }

    void Robot::teleport(concord::Pose pose) {
        teleport(pose, true);
    }
    
    void Robot::teleport(concord::Pose pose, bool propagate) {
        spdlog::info("Teleporting robot {} to ({:.2f}, {:.2f}) - breaking chain connections", info.name, pose.point.x, pose.point.y);
        control_system->reset_controls();
        
        // Break all chain connections before teleporting
        if (propagate) {
            chain_manager->break_chain_for_teleport();
        }
        
        chassis->teleport(pose);
    }

    void Robot::respawn() {
        spdlog::info("Respawning robot {} - breaking chain connections", info.name);
        control_system->reset_controls();
        
        // Break all chain connections before respawning
        chain_manager->break_chain_for_teleport();
        
        chassis->teleport(spawn_position);
    }

    void Robot::update_color(const pigment::RGB& new_color) {
        info.color = new_color;
        if (chassis) {
            chassis->update_color(new_color);
        }
    }

    void Robot::update(float angular, float linear) {
        set_angular(angular);
        set_linear(linear);
    }

    // Connection management - implement delegation methods
    bool Robot::try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>>& all_robots) {
        return chain_manager->try_connect_nearby_slave(all_robots);
    }

    bool Robot::try_connect_nearby() {
        return chain_manager->try_connect_nearby();
    }

    bool Robot::try_connect_from_chain_end() {
        return chain_manager->try_connect_from_chain_end();
    }

    void Robot::disconnect_trailer() {
        chain_manager->disconnect_trailer();
    }

    void Robot::disconnect_all_followers() {
        chain_manager->disconnect_all_followers();
    }

    void Robot::disconnect_last_follower() {
        chain_manager->disconnect_last_follower();
    }

    void Robot::disconnect_at_position(int position) {
        chain_manager->disconnect_at_position(position);
    }

    void Robot::disconnect_from_position(int position) {
        chain_manager->disconnect_from_position(position);
    }

    bool Robot::is_connected() const {
        return chain_manager->is_connected();
    }

    // Chain management - implement delegation methods
    std::vector<Robot*> Robot::get_connected_followers() const {
        return chain_manager->get_connected_followers();
    }

    Robot* Robot::get_master_robot() const {
        return chain_manager->get_master_robot();
    }

    bool Robot::is_follower() const {
        return chain_manager->is_follower();
    }

    Robot* Robot::get_root_master() const {
        return chain_manager->get_root_master();
    }

    std::vector<Robot*> Robot::get_full_chain() const {
        return chain_manager->get_full_chain();
    }

    int Robot::get_chain_length() const {
        return chain_manager->get_chain_length();
    }

    int Robot::get_position_in_chain() const {
        return chain_manager->get_position_in_chain();
    }

    void Robot::print_chain_status() const {
        chain_manager->print_chain_status();
    }

    // Capability management - implement delegation methods
    void Robot::update_follower_capabilities() {
        chain_manager->update_follower_capabilities();
    }

    const FollowerCapabilities& Robot::get_follower_capabilities() const {
        return chain_manager->get_follower_capabilities();
    }

    bool Robot::has_steering_capability() const {
        return chain_manager->has_steering_capability();
    }

    bool Robot::has_throttle_capability() const {
        return chain_manager->has_throttle_capability();
    }

    bool Robot::has_available_master_hitches() const {
        return chain_manager->has_available_master_hitches();
    }

// ============================================================================
// ControlSystem Implementation
// ============================================================================

void ControlSystem::init(const RobotInfo& robo) {
    steerings.resize(robo.wheels.size(), 0.0f);
    steerings_max = robo.controls.steerings_max;
    steerings_diff = robo.controls.steerings_diff;
    throttles.resize(robo.wheels.size(), 0.0f);
    throttles_max = robo.controls.throttles_max;
}

void ControlSystem::reset_controls() {
    for (uint i = 0; i < steerings.size(); ++i) {
        steerings[i] = 0.0f;
    }
    for (uint i = 0; i < throttles.size(); ++i) {
        throttles[i] = 0.0f;
    }
}

void ControlSystem::set_angular(float angular) {
    constexpr float in_min = -1.0f, in_max = 1.0f;
    const float sign = (angular < 0.0f ? -1.0f : 1.0f);
    for (size_t i = 0; i < steerings.size(); ++i) {
        float o1 = steerings_max[i] - sign * steerings_diff[i];
        float o2 = -steerings_max[i] + sign * steerings_diff[i];
        steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
    }
    
    // Propagate to followers that have steering capability
    for (Robot* follower : robot->chain_manager->get_connected_followers()) {
        if (follower->has_steering_capability()) {
            follower->set_angular_as_follower(angular, *robot);
        }
    }
}

void ControlSystem::set_linear(float linear) {
    constexpr float in_min = -1.0f, in_max = 1.0f;
    for (uint i = 0; i < throttles.size(); ++i) {
        auto lin_val = linear;
        if (steerings[i] > 0.0f && robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        } else if (steerings[i] < 0.0f && !robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        }
        throttles[i] = utils::mapper(lin_val, in_min, in_max, throttles_max[i], -throttles_max[i]);
    }
    
    // Propagate to followers that have throttle capability
    for (Robot* follower : robot->chain_manager->get_connected_followers()) {
        if (follower->has_throttle_capability()) {
            follower->set_linear_as_follower(linear, *robot);
        }
    }
}

void ControlSystem::set_angular_as_follower(float angular, const Robot& master) {
    // Only apply if this robot has steering capability and is actually a follower
    if (!robot->chain_manager->has_steering_capability() || robot->role != RobotRole::FOLLOWER) {
        return;
    }
    
    // Apply the same steering logic as master, but potentially with different scaling
    constexpr float in_min = -1.0f, in_max = 1.0f;
    const float sign = (angular < 0.0f ? -1.0f : 1.0f);
    for (size_t i = 0; i < steerings.size(); ++i) {
        float o1 = steerings_max[i] - sign * steerings_diff[i];
        float o2 = -steerings_max[i] + sign * steerings_diff[i];
        steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
    }
    
    // Continue propagation to any followers this robot might have
    for (Robot* follower : robot->chain_manager->get_connected_followers()) {
        if (follower->has_steering_capability()) {
            follower->set_angular_as_follower(angular, *robot);
        }
    }
}

void ControlSystem::set_linear_as_follower(float linear, const Robot& master) {
    // Only apply if this robot has throttle capability and is actually a follower
    if (!robot->chain_manager->has_throttle_capability() || robot->role != RobotRole::FOLLOWER) {
        return;
    }
    
    // Apply the same throttle logic as master, but potentially with different scaling
    constexpr float in_min = -1.0f, in_max = 1.0f;
    for (uint i = 0; i < throttles.size(); ++i) {
        auto lin_val = linear;
        if (steerings[i] > 0.0f && robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        } else if (steerings[i] < 0.0f && !robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        }
        throttles[i] = utils::mapper(lin_val, in_min, in_max, throttles_max[i], -throttles_max[i]);
    }
    
    // Continue propagation to any followers this robot might have
    for (Robot* follower : robot->chain_manager->get_connected_followers()) {
        if (follower->has_throttle_capability()) {
            follower->set_linear_as_follower(linear, *robot);
        }
    }
}

// ============================================================================
// ChainManager Implementation
// ============================================================================

bool ChainManager::try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>>& all_robots) {
    // Only MASTER robots can initiate connections
    if (robot->role != RobotRole::MASTER || !robot->info.hitches.count("rear_hitch")) {
        return false;
    }
    
    // Already connected (check if we have any followers)
    if (!connected_followers.empty()) {
        return false;
    }
    
    auto my_pos = robot->get_position().point;
    float connection_range = 0.1f; // Hitch points must be overlapping
    
    for (const auto &other_robot : all_robots) {
        // Skip self and non-slaves
        if (other_robot.get() == robot || other_robot->role != RobotRole::SLAVE) {
            continue;
        }
        
        // Check if slave has front hitch
        if (!other_robot->info.hitches.count("front_hitch")) {
            continue;
        }
        
        // Calculate actual world positions of hitch points
        auto my_rear_hitch_pos = robot->info.hitches["rear_hitch"];
        auto slave_front_hitch_pos = other_robot->info.hitches["front_hitch"];
        auto other_pos = other_robot->get_position().point;
        auto my_angle = robot->get_position().angle.yaw;
        auto other_angle = other_robot->get_position().angle.yaw;
        
        // Transform hitch positions to world coordinates considering robot rotation
        float my_hitch_world_x = my_pos.x + my_rear_hitch_pos.bound.pose.point.x * cos(my_angle) -
                                 my_rear_hitch_pos.bound.pose.point.y * sin(my_angle);
        float my_hitch_world_y = my_pos.y + my_rear_hitch_pos.bound.pose.point.x * sin(my_angle) +
                                 my_rear_hitch_pos.bound.pose.point.y * cos(my_angle);
        
        float other_hitch_world_x = other_pos.x + slave_front_hitch_pos.bound.pose.point.x * cos(other_angle) -
                                    slave_front_hitch_pos.bound.pose.point.y * sin(other_angle);
        float other_hitch_world_y = other_pos.y + slave_front_hitch_pos.bound.pose.point.x * sin(other_angle) +
                                    slave_front_hitch_pos.bound.pose.point.y * cos(other_angle);
        
        // Check if hitch points are overlapping (very close)
        float hitch_dist = std::sqrt(std::pow(my_hitch_world_x - other_hitch_world_x, 2) +
                                     std::pow(my_hitch_world_y - other_hitch_world_y, 2));
        
        if (hitch_dist <= connection_range) {
            // Use the midpoint between the two hitch points as the joint position
            muli::Vec2 hitch_world_pos((my_hitch_world_x + other_hitch_world_x) / 2.0f,
                                       (my_hitch_world_y + other_hitch_world_y) / 2.0f);
            
            auto new_joint = robot->world->CreateRevoluteJoint(robot->chassis->get_body(), 
                                                               other_robot->chassis->get_body(), 
                                                               hitch_world_pos,
                                                               20.0f, // frequency
                                                               0.8f,  // damping
                                                               10.0f  // joint mass
            );
            
            if (new_joint) {
                // Add to followers using new system
                add_follower(other_robot.get(), new_joint);
                other_robot->chain_manager->set_master_robot(robot);
                other_robot->role = RobotRole::FOLLOWER; // Change slave to follower
                
                // Follower adopts master's color
                other_robot->update_color(robot->info.color);
                
                // Update capabilities for both robots
                update_follower_capabilities();
                other_robot->update_follower_capabilities();
                
                spdlog::info("Connected {} to {}", robot->info.name, other_robot->info.name);
                return true;
            }
        }
    }
    
    return false;
}

bool ChainManager::try_connect_nearby() {
    // Only MASTER robots or FOLLOWERS with available master hitches can initiate connections
    if ((robot->role != RobotRole::MASTER && robot->role != RobotRole::FOLLOWER) || !robot->chassis) {
        return false;
    }
    
    // Check if this robot has any available master hitch
    bool has_available_master_hitch = false;
    for (const auto &hitch : robot->chassis->hitches) {
        if (hitch.is_master) {
            // Check if this hitch is already used
            bool hitch_used = false;
            for (size_t i = 0; i < connected_followers.size(); ++i) {
                // This is a simplified check - in reality we'd need to track which hitch was used
                // For now, assume only one master hitch per robot
                hitch_used = true;
                break;
            }
            if (!hitch_used) {
                has_available_master_hitch = true;
                break;
            }
        }
    }
    
    if (!has_available_master_hitch) {
        return false;
    }
    
    auto all_robots = robot->get_all_robots();
    float connection_range = 2.0f; // Increased range for easier connection
    
    for (Robot *other_robot : all_robots) {
        // Skip self and non-slaves
        if (other_robot == robot || other_robot->role != RobotRole::SLAVE) {
            continue;
        }
        
        // Skip if other robot has no chassis or hitches
        if (!other_robot->chassis || other_robot->chassis->hitches.empty()) {
            continue;
        }
        
        // Try to connect my master hitches to their slave hitches
        for (const auto &my_hitch : robot->chassis->hitches) {
            for (const auto &other_hitch : other_robot->chassis->hitches) {
                // Only connect master hitch to slave hitch
                if (!my_hitch.is_master || other_hitch.is_master) {
                    continue;
                }
                
                // Get actual world positions of hitches (updated by tick)
                concord::Point my_hitch_pos = my_hitch.pose.point;
                concord::Point other_hitch_pos = other_hitch.pose.point;
                
                // Calculate distance between hitches
                float hitch_dist = std::sqrt(std::pow(my_hitch_pos.x - other_hitch_pos.x, 2) +
                                             std::pow(my_hitch_pos.y - other_hitch_pos.y, 2));
                
                if (hitch_dist <= connection_range) {
                    // Use the midpoint between the two hitch points as the joint position
                    muli::Vec2 hitch_world_pos((my_hitch_pos.x + other_hitch_pos.x) / 2.0f,
                                               (my_hitch_pos.y + other_hitch_pos.y) / 2.0f);
                    
                    auto new_joint = robot->world->CreateRevoluteJoint(robot->chassis->get_body(),
                                                                       other_robot->chassis->get_body(), 
                                                                       hitch_world_pos,
                                                                       20.0f, // frequency
                                                                       0.8f,  // damping
                                                                       10.0f  // joint mass
                    );
                    
                    if (new_joint) {
                        // Add to followers list
                        add_follower(other_robot, new_joint);
                        
                        // Set backward reference
                        other_robot->chain_manager->set_master_robot(robot);
                        other_robot->role = RobotRole::FOLLOWER;
                        
                        // Follower adopts master's color
                        other_robot->update_color(robot->info.color);
                        
                        // Update capabilities for both robots
                        update_follower_capabilities();
                        other_robot->update_follower_capabilities();
                        
                        spdlog::info("Connected {} to {} (hitch distance: {:.2f}m)", robot->info.name,
                                     other_robot->info.name, hitch_dist);
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}

bool ChainManager::try_connect_from_chain_end() {
    // Find the end of the chain starting from this robot
    Robot* chain_end = robot;
    
    // Follow the chain to the end (robot with no followers)
    while (!chain_end->chain_manager->connected_followers.empty()) {
        chain_end = chain_end->chain_manager->connected_followers.back();  // Follow the last follower
    }
    
    // Try to connect from the chain end
    spdlog::info("Trying to connect from chain end: {}", chain_end->info.name);
    return chain_end->try_connect_nearby();
}

void ChainManager::disconnect_trailer() {
    disconnect_all_followers();
}

void ChainManager::disconnect_all_followers() {
    for (size_t i = 0; i < connected_followers.size(); ++i) {
        Robot* follower = connected_followers[i];
        muli::RevoluteJoint* joint = connection_joints[i];
        
        if (joint) {
            robot->world->Destroy(joint);
        }
        
        if (follower) {
            follower->role = RobotRole::SLAVE;  // Change back to slave
            follower->chain_manager->master_robot = nullptr;
            
            // Restore original color when disconnecting
            follower->update_color(follower->original_color);
            
            spdlog::info("Disconnected {} from {}", follower->info.name, robot->info.name);
            
            // Recursively disconnect any sub-followers
            follower->disconnect_all_followers();
            
            // Update capabilities after disconnection
            follower->update_follower_capabilities();
        }
    }
    
    connected_followers.clear();
    connection_joints.clear();
    
    // Update our own capabilities after disconnection
    update_follower_capabilities();
}

// ChainManager methods already implemented above...

    // Additional core Robot methods that were moved from header
    void Robot::add_sensor(std::unique_ptr<Sensor> sensor) {
        sensors.push_back(std::move(sensor));
    }

    Sensor* Robot::get_sensor(const std::string& type) const {
        for (const auto& sensor : sensors) {
            if (!sensor) {
                continue; // Skip null sensors
            }
            if (sensor->get_type() == type) {
                return sensor.get();
            }
        }
        return nullptr;
    }

    // Spatial queries - robot can find other robots
    std::vector<Robot*> Robot::get_all_robots() const {
        if (!simulator) {
            return {};
        }
        return simulator->get_all_robots();
    }

    Robot* Robot::get_closest_robot(float max_distance) const {
        if (!simulator) {
            return nullptr;
        }
        auto all_robots = get_all_robots();
        Robot* closest = nullptr;
        float min_dist = max_distance;
        
        auto my_pos = get_position().point;
        for (Robot* other : all_robots) {
            if (other == this) continue;
            
            auto other_pos = other->get_position().point;
            float dist = std::sqrt(std::pow(my_pos.x - other_pos.x, 2) + std::pow(my_pos.y - other_pos.y, 2));
            
            if (dist < min_dist) {
                min_dist = dist;
                closest = other;
            }
        }
        
        return closest;
    }

    void Robot::visualize_once() {
        // Implementation for one-time visualization setup
        if (!rec) {
            return;
        }
        
        // Log initial robot state and setup
        rec->log(info.name + "/setup", rerun::TextLog("Robot " + info.name + " initialized"));
    }

    void Robot::visualize() {
        if (!rec) {
            return;
        }

        // Create label with role prefix and power percentage
        std::string role_prefix;
        switch (role) {
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
        
        std::string label = role_prefix + info.name;
        if (has_power()) {
            int percentage = static_cast<int>(get_power_percentage());
            label += "(" + std::to_string(percentage) + "%)";
        }
        
        // Visualize chassis (this was missing in the refactored version!)
        if (chassis) {
            chassis->visualize(label);
        }

        auto x = this->info.bound.pose.point.x;
        auto y = this->info.bound.pose.point.y;

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(info.color.r, info.color.g, info.color.b));

        // 3D position visualization
        std::vector<rerun::components::Position3D> positions = {
            rerun::components::Position3D(float(x), float(y), 0.1f)};
        rec->log_static(this->info.name + "/pose", rerun::Points3D(positions).with_colors(colors));

        // GPS coordinates visualization
        auto wgs_coords = this->info.bound.pose.point.toWGS(datum);
        auto lat = float(wgs_coords.lat);
        auto lon = float(wgs_coords.lon);
        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));
        rec->log_static(this->info.name + "/pose", rerun::GeoPoints(locators).with_colors(colors));

        // Visualize pulse with proper size calculation
        visualize_pulse(std::max(info.bound.size.x, info.bound.size.y) * 3.0f);
    }

    void Robot::visualize_pulse(float p_s, float gps_mult, float inc) {
        concord::Point point(this->info.bound.pose.point.x, this->info.bound.pose.point.y, 0.0f);
        if (!pulsing) {
            return;
        }

        // visualize local pulse
        std::vector<rerun::Vec3D> poi;
        auto pulse_enu_size = pulse_enu.getRadius() + inc;
        if (pulse_enu.getRadius() > p_s) {
            pulsing = false;
            pulse_enu_size = 0.0;
        }
        pulse_enu = concord::Circle(point, pulse_enu_size);
        auto pointss = pulse_enu.as_polygon(50);
        for (auto &point : pointss) {
            poi.push_back({float(point.x), float(point.y), 0.0f});
        }
        poi.push_back({float(pointss[0].x), float(pointss[0].y), 0.0f});
        rec->log_static(
            this->info.name + "/pulse/enu",
            rerun::LineStrips3D({{poi}})
                .with_colors({{rerun::Color(info.color.r, info.color.g, info.color.b)}})
                .with_radii({{float(utils::mapper(
                    pulse_enu_size, 0.0, std::max(info.bound.size.x, info.bound.size.y) * 3.0f, 0.03, 0.0005))}}));

        // visualize gps pulse
        std::vector<rerun::LatLon> locators;
        auto pulse_gps_size = pulse_gps.getRadius() + inc * gps_mult;
        if (pulse_gps.getRadius() > p_s * gps_mult) {
            pulsing = false;
            pulse_gps_size = 0.0;
        }
        pulse_gps = concord::Circle(point, pulse_gps_size);
        auto pointss_gps = pulse_gps.as_polygon(50);
        for (auto &point : pointss_gps) {
            auto wgs_coords = point.toWGS(datum);
            locators.push_back(rerun::LatLon(wgs_coords.lat, wgs_coords.lon));
        }
        auto first_wgs_coords = pointss_gps[0].toWGS(datum);
        locators.push_back(rerun::LatLon(first_wgs_coords.lat, first_wgs_coords.lon));
        auto linestr = rerun::components::GeoLineString::from_lat_lon(locators);
        rec->log_static(this->info.name + "/pulse/wgs",
                        rerun::GeoLineStrings(linestr)
                            .with_colors(rerun::Color(info.color.r, info.color.g, info.color.b))
                            .with_radii({{float(utils::mapper(
                                pulse_gps_size, 0.0, std::max(info.bound.size.x, info.bound.size.y) * 3.0f * gps_mult,
                                0.03 * gps_mult, 0.0005 * gps_mult))}}));
    }

// Add missing ChainManager method implementations

void ChainManager::disconnect_last_follower() {
    // Find the end of the chain and disconnect the last follower
    Robot* chain_end = robot;
    Robot* previous_robot = nullptr;
    
    // Traverse to the end of the chain
    while (!chain_end->chain_manager->connected_followers.empty()) {
        previous_robot = chain_end;
        chain_end = chain_end->chain_manager->connected_followers.back();  // Follow the chain
    }
    
    // If we found a chain end that isn't ourselves, disconnect it from its master
    if (chain_end != robot && previous_robot) {
        // Find the connection to disconnect
        auto it = std::find(previous_robot->chain_manager->connected_followers.begin(), 
                           previous_robot->chain_manager->connected_followers.end(), 
                           chain_end);
        
        if (it != previous_robot->chain_manager->connected_followers.end()) {
            size_t index = std::distance(previous_robot->chain_manager->connected_followers.begin(), it);
            
            // Destroy the physics joint
            if (index < previous_robot->chain_manager->connection_joints.size()) {
                muli::RevoluteJoint* joint = previous_robot->chain_manager->connection_joints[index];
                if (joint) {
                    previous_robot->world->Destroy(joint);
                }
                previous_robot->chain_manager->connection_joints.erase(previous_robot->chain_manager->connection_joints.begin() + index);
            }
            
            // Remove from followers list
            previous_robot->chain_manager->connected_followers.erase(it);
            
            // Reset disconnected robot's state
            chain_end->role = RobotRole::SLAVE;
            chain_end->chain_manager->master_robot = nullptr;
            
            // Restore original color when disconnecting
            chain_end->update_color(chain_end->original_color);
            
            chain_end->update_follower_capabilities();
            
            // Update master's capabilities
            previous_robot->update_follower_capabilities();
            
            spdlog::info("Disconnected last follower {} from {}", 
                       chain_end->info.name, previous_robot->info.name);
        }
    }
}

bool ChainManager::is_connected() const {
    return !connected_followers.empty() || master_robot != nullptr;
}

Robot* ChainManager::get_root_master() const {
    const Robot* current = robot;
    while (current->chain_manager->master_robot != nullptr) {
        current = current->chain_manager->master_robot;
    }
    return const_cast<Robot*>(current);
}

std::vector<Robot*> ChainManager::get_full_chain() const {
    std::vector<Robot*> chain;
    
    // Start from root master
    Robot* root = get_root_master();
    chain.push_back(root);
    
    // Follow the chain adding all followers
    std::function<void(Robot*)> add_followers = [&](Robot* current) {
        for (Robot* follower : current->chain_manager->connected_followers) {
            chain.push_back(follower);
            add_followers(follower);  // Recursively add sub-followers
        }
    };
    
    add_followers(root);
    return chain;
}

int ChainManager::get_chain_length() const {
    return static_cast<int>(get_full_chain().size());
}

int ChainManager::get_position_in_chain() const {
    auto chain = get_full_chain();
    for (int i = 0; i < static_cast<int>(chain.size()); ++i) {
        if (chain[i] == robot) {
            return i;
        }
    }
    return -1;  // Not found (shouldn't happen)
}

void ChainManager::print_chain_status() const {
    auto chain = get_full_chain();
    std::cout << "Chain from " << get_root_master()->info.name << " (length=" << chain.size() << "): ";
    for (int i = 0; i < static_cast<int>(chain.size()); ++i) {
        std::cout << i << ":" << chain[i]->info.name << "("
                  << (chain[i]->role == RobotRole::MASTER     ? "M"
                      : chain[i]->role == RobotRole::FOLLOWER ? "F"
                                                              : "S")
                  << ")";
        if (i < static_cast<int>(chain.size()) - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;
}

void ChainManager::update_follower_capabilities() {
    // Reset capabilities
    follower_capabilities.has_steering = false;
    follower_capabilities.has_throttle = false;
    follower_capabilities.has_tank = robot->tank.has_value();
    follower_capabilities.has_additional_hitches = false;
    follower_capabilities.available_master_hitches.clear();
    
    // Check for steering capability
    for (const auto &max_angle : robot->control_system->get_steerings_max()) {
        if (max_angle > 0) {
            follower_capabilities.has_steering = true;
            break;
        }
    }
    
    // Check for throttle capability
    for (const auto &max_throttle : robot->control_system->get_throttles_max()) {
        if (max_throttle > 0) {
            follower_capabilities.has_throttle = true;
            break;
        }
    }
    
    // Check for available master hitches (for chaining)
    if (robot->chassis) {
        for (const auto &hitch : robot->chassis->hitches) {
            if (hitch.is_master) {
                // Better hitch occupation detection
                bool hitch_occupied = false;
                
                // Check the specific hitch - if it's a rear master hitch on a trailer,
                // it should be available even if the trailer is a follower
                if (hitch.name == "rear_hitch" && robot->info.type == "trailer") {
                    // Rear hitch on trailer - available unless we already have a follower
                    hitch_occupied = (connected_followers.size() >= 1);
                } else {
                    // For other hitches (like tractor rear_hitch), use simpler logic
                    hitch_occupied = (connected_followers.size() >= 1);
                }
                
                if (!hitch_occupied) {
                    follower_capabilities.available_master_hitches.push_back(hitch.name);
                    follower_capabilities.has_additional_hitches = true;
                }
            }
        }
    }
    
    // Log capabilities for debugging
    spdlog::debug("{} capabilities: steering={}, throttle={}, tank={}, master_hitches={}", robot->info.name,
                  follower_capabilities.has_steering, follower_capabilities.has_throttle,
                  follower_capabilities.has_tank, follower_capabilities.available_master_hitches.size());
}

void ChainManager::disconnect_at_position(int position) {
    // Position 1 = first follower, 2 = second follower, etc.
    if (position < 1) {
        spdlog::warn("Invalid position {} for disconnection (must be >= 1)", position);
        return;
    }
    
    auto chain = get_full_chain();
    if (position >= static_cast<int>(chain.size())) {
        spdlog::warn("Position {} is beyond chain length {}", position, chain.size());
        return;
    }
    
    Robot* robot_to_disconnect = chain[position];
    Robot* its_master = robot_to_disconnect->chain_manager->master_robot;
    
    if (!its_master) {
        spdlog::warn("Robot at position {} has no master (might be root)", position);
        return;
    }
    
    // Find the connection in the master's followers list
    auto it = std::find(its_master->chain_manager->connected_followers.begin(), 
                       its_master->chain_manager->connected_followers.end(), 
                       robot_to_disconnect);
    
    if (it != its_master->chain_manager->connected_followers.end()) {
        size_t index = std::distance(its_master->chain_manager->connected_followers.begin(), it);
        
        // Destroy the physics joint
        if (index < its_master->chain_manager->connection_joints.size()) {
            muli::RevoluteJoint* joint = its_master->chain_manager->connection_joints[index];
            if (joint) {
                its_master->world->Destroy(joint);
            }
            its_master->chain_manager->connection_joints.erase(its_master->chain_manager->connection_joints.begin() + index);
        }
        
        // Remove from followers list
        its_master->chain_manager->connected_followers.erase(it);
        
        // Reset disconnected robot's state (this will also disconnect its followers)
        robot_to_disconnect->role = RobotRole::SLAVE;
        robot_to_disconnect->chain_manager->master_robot = nullptr;
        
        // Restore original color when disconnecting
        robot_to_disconnect->update_color(robot_to_disconnect->original_color);
        
        robot_to_disconnect->disconnect_all_followers();  // Disconnect everything after this point
        robot_to_disconnect->update_follower_capabilities();
        
        // Update master's capabilities
        its_master->update_follower_capabilities();
        
        spdlog::info("Disconnected robot at position {} ({}) from {}", 
                   position, robot_to_disconnect->info.name, its_master->info.name);
    }
}

void ChainManager::disconnect_from_position(int position) {
    // Disconnect everything from position onwards
    if (position < 1) {
        spdlog::warn("Invalid position {} for disconnection (must be >= 1)", position);
        return;
    }
    
    auto chain = get_full_chain();
    if (position >= static_cast<int>(chain.size())) {
        spdlog::warn("Position {} is beyond chain length {}", position, chain.size());
        return;
    }
    
    // Just disconnect at the position - the disconnect_all_followers() call in disconnect_at_position
    // will handle disconnecting everything after that point
    disconnect_at_position(position);
    
    spdlog::info("Disconnected chain from position {} onwards", position);
}

void ChainManager::break_chain_for_teleport() {
    spdlog::info("Breaking chain and teleporting all robots to spawn for robot {}", robot->info.name);
    
    // Get the root master and the full chain
    Robot* root_master = get_root_master();
    auto full_chain = root_master->chain_manager->get_full_chain();
    
    spdlog::info("Full chain has {} robots, starting from root {}", full_chain.size(), root_master->info.name);
    
    // Step 1: Disconnect all connections in the entire chain
    root_master->chain_manager->disconnect_all_followers();
    
    // Step 2: Teleport each robot individually to its spawn position
    for (Robot* chain_robot : full_chain) {
        if (chain_robot != robot) {  // Don't teleport the initiating robot here
            spdlog::info("Teleporting chain robot {} to its spawn position", chain_robot->info.name);
            chain_robot->teleport(chain_robot->get_spawn_position(), false);  // No propagation
            
            // Small delay to prevent physics conflicts
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

} // namespace fs
