#include "flatsim/robot.hpp"
#include "flatsim/simulator.hpp"
#include <algorithm>
#include <cmath>

namespace fs {

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
            tank->tick(dt, chassis->get_pose());
        }

        // visualize();
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

    void Robot::reset_controls() { control_system->reset_controls(); }

    void Robot::set_angular(float angular) { control_system->set_angular(angular); }

    void Robot::set_linear(float linear) { control_system->set_linear(linear); }

    void Robot::set_angular_as_follower(float angular, const Robot &master) {
        control_system->set_angular_as_follower(angular, master);
    }

    void Robot::set_linear_as_follower(float linear, const Robot &master) {
        control_system->set_linear_as_follower(linear, master);
    }

    void Robot::teleport(concord::Pose pose) { teleport(pose, true); }

    void Robot::teleport(concord::Pose pose, bool propagate) {
        spdlog::info("Teleporting robot {} to ({:.2f}, {:.2f}) - breaking chain connections", info.name, pose.point.x,
                     pose.point.y);
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

    void Robot::update_color(const pigment::RGB &new_color) {
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
    bool Robot::try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>> &all_robots) {
        return chain_manager->try_connect_nearby_slave(all_robots);
    }

    bool Robot::try_connect_nearby() { return chain_manager->try_connect_nearby(); }

    bool Robot::try_connect_from_chain_end() { return chain_manager->try_connect_from_chain_end(); }

    void Robot::disconnect_trailer() { chain_manager->disconnect_trailer(); }

    void Robot::disconnect_all_followers() { chain_manager->disconnect_all_followers(); }

    void Robot::disconnect_last_follower() { chain_manager->disconnect_last_follower(); }

    void Robot::disconnect_at_position(int position) { chain_manager->disconnect_at_position(position); }

    void Robot::disconnect_from_position(int position) { chain_manager->disconnect_from_position(position); }

    bool Robot::is_connected() const { return chain_manager->is_connected(); }

    // Chain management - implement delegation methods
    std::vector<Robot *> Robot::get_connected_followers() const { return chain_manager->get_connected_followers(); }

    Robot *Robot::get_master_robot() const { return chain_manager->get_master_robot(); }

    bool Robot::is_follower() const { return chain_manager->is_follower(); }

    Robot *Robot::get_root_master() const { return chain_manager->get_root_master(); }

    std::vector<Robot *> Robot::get_full_chain() const { return chain_manager->get_full_chain(); }

    int Robot::get_chain_length() const { return chain_manager->get_chain_length(); }

    int Robot::get_position_in_chain() const { return chain_manager->get_position_in_chain(); }

    void Robot::print_chain_status() const { chain_manager->print_chain_status(); }

    // Capability management - implement delegation methods
    void Robot::update_follower_capabilities() { chain_manager->update_follower_capabilities(); }

    const FollowerCapabilities &Robot::get_follower_capabilities() const {
        return chain_manager->get_follower_capabilities();
    }

    bool Robot::has_steering_capability() const { return chain_manager->has_steering_capability(); }

    bool Robot::has_throttle_capability() const { return chain_manager->has_throttle_capability(); }

    bool Robot::has_available_master_hitches() const { return chain_manager->has_available_master_hitches(); }

    // Additional core Robot methods
    void Robot::add_sensor(std::unique_ptr<Sensor> sensor) { sensors.push_back(std::move(sensor)); }

    Sensor *Robot::get_sensor(const std::string &type) const {
        for (const auto &sensor : sensors) {
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
            float dist = std::sqrt(std::pow(my_pos.x - other_pos.x, 2) + std::pow(my_pos.y - other_pos.y, 2));

            if (dist < min_dist) {
                min_dist = dist;
                closest = other;
            }
        }

        return closest;
    }

    void Robot::tock() {
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
        if (has_power()) label += "(" + std::to_string(static_cast<int>(get_power_percentage())) + "%)";
        if (chassis) chassis->tock(label);

        // Visualize tank if present
        if (tank.has_value()) {
            tank->tock(rec);
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
    }

    void Robot::visualize_pulse(float p_s, float gps_mult, float inc) {
        if (!pulsing) {
            return;
        }

        // Simple pulse implementation - just log basic pulse state
        if (rec) {
            auto pos = get_position();
            rec->log(info.name + "/pulse", rerun::Points2D({rerun::Position2D(pos.point.x, pos.point.y)})
                                               .with_colors({rerun::Color(255, 255, 255, 200)})
                                               .with_radii({2.0f}));
        }

        // Reset pulsing after some time
        pulsing = false;
    }

} // namespace fs
