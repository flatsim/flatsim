#include "flatsim/robot.hpp"
#include "flatsim/simulator.hpp"
#include <cmath>

namespace fs {

    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
        : rec(rec), world(world) {
        filter.group = 0;                    // Use bit/mask system, not group system
        filter.bit = 1 << group;             // Each robot gets unique bit position
        filter.mask = ~(1 << group);         // Exclude own bit from collision mask
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
        chassis->update(steerings, throttles, dt);

        // Update power consumption based on operation mode
        if (power && *power && is_powered()) {
            float consumption_multiplier = 1.0f;
            switch (mode) {
                case OP::IDLE:
                    consumption_multiplier = 0.1f;  // Minimal consumption when idle
                    break;
                case OP::TRANSPORT:
                    consumption_multiplier = 0.5f;  // Medium consumption when moving
                    break;
                case OP::WORK:
                    consumption_multiplier = 1.5f;  // Higher consumption when working
                    break;
                case OP::CHARGING:
                    consumption_multiplier = 0.0f;  // No consumption when charging
                    (*power)->charge(dt); // Charge battery
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
        this->role = robo.role;  // Set role from RobotInfo
        this->spawn_position = robo.bound.pose;

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

        steerings.resize(robo.wheels.size(), 0.0f);
        steerings_max = robo.controls.steerings_max;
        steerings_diff = robo.controls.steerings_diff;
        throttles.resize(robo.wheels.size(), 0.0f);
        throttles_max = robo.controls.throttles_max;

        // Initialize tank if present
        if (robo.tank.has_value()) {
            tank = Tank(robo.tank->name, Tank::Type::HARVEST, 
                       robo.tank->capacity, 0.0f, 0.0f);
            tank->init(info.color, info.name, robo.tank->bound);
        }

        // Initialize power source if present
        if (robo.power_source.has_value()) {
            auto power_type = (robo.power_source->type == PowerType::BATTERY) ? 
                Power::Type::BATTERY : Power::Type::FUEL;
            power = std::make_unique<Power>(
                robo.power_source->name, power_type, 
                robo.power_source->capacity, robo.power_source->consumption_rate,
                robo.power_source->charge_rate
            );
        }
    }

    void Robot::reset_controls() {
        for (uint i = 0; i < steerings.size(); ++i) {
            steerings[i] = 0.0f;
        }
        for (uint i = 0; i < throttles.size(); ++i) {
            throttles[i] = 0.0f;
        }
    }

    void Robot::set_angular(float angular) {
        constexpr float in_min = -1.0f, in_max = 1.0f;
        const float sign = (angular < 0.0f ? -1.0f : 1.0f);
        for (size_t i = 0; i < steerings.size(); ++i) {
            float o1 = steerings_max[i] - sign * steerings_diff[i];
            float o2 = -steerings_max[i] + sign * steerings_diff[i];
            steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
        }
    }

    void Robot::set_linear(float linear) {
        constexpr float in_min = -1.0f, in_max = 1.0f;
        for (uint i = 0; i < throttles.size(); ++i) {
            auto lin_val = linear;
            if (steerings[i] > 0.0f && info.controls.left_side[i]) {
                auto proportion = utils::ackermann_scale(steerings[i], info.bound.size.x);
                lin_val = linear * proportion;
            } else if (steerings[i] < 0.0f && !info.controls.left_side[i]) {
                auto proportion = utils::ackermann_scale(steerings[i], info.bound.size.x);
                lin_val = linear * proportion;
            }
            throttles[i] = utils::mapper(lin_val, in_min, in_max, throttles_max[i], -throttles_max[i]);
        }
    }

    void Robot::teleport(concord::Pose pose) {
        reset_controls();
        chassis->teleport(pose);
    }
    void Robot::respawn() {
        reset_controls();
        chassis->teleport(spawn_position);
    }

    void Robot::visualize() {
        // Create label with role prefix and power percentage
        std::string role_prefix;
        switch (role) {
            case RobotRole::MASTER:
                role_prefix = "(M)";
                break;
            case RobotRole::SLAVE:
                role_prefix = "(S)";
                break;
            case RobotRole::FOLLOWER:
                role_prefix = "(F)";
                break;
            default:
                role_prefix = "(?)";
                break;
        }
        
        std::string label = role_prefix + info.name;
        if (has_power()) {
            int percentage = static_cast<int>(get_power_percentage());
            label += "(" + std::to_string(percentage) + "%)";
        }
        
        chassis->visualize(label);

        auto x = this->info.bound.pose.point.x;
        auto y = this->info.bound.pose.point.y;

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(info.color.r, info.color.g, info.color.b));

        std::vector<rerun::components::Position3D> positions = {
            rerun::components::Position3D(float(x), float(y), 0.1f)};
        rec->log_static(this->info.name + "/pose", rerun::Points3D(positions).with_colors(colors));

        auto wgs_coords = this->info.bound.pose.point.toWGS(datum);
        auto lat = float(wgs_coords.lat);
        auto lon = float(wgs_coords.lon);
        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));
        rec->log_static(this->info.name + "/pose", rerun::GeoPoints(locators).with_colors(colors));

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

    // Sensor management methods
    void Robot::add_sensor(std::unique_ptr<Sensor> sensor) { sensors.push_back(std::move(sensor)); }

    Sensor *Robot::get_sensor(const std::string &type) const {
        for (const auto &sensor : sensors) {
            if (sensor->get_type() == type) {
                return sensor.get();
            }
        }
        return nullptr;
    }

    bool Robot::try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>>& all_robots) {
        // Only MASTER robots can initiate connections
        if (role != RobotRole::MASTER || !info.hitches.count("rear_hitch")) {
            return false;
        }
        
        // Already connected (check if we have any followers)
        if (!connected_followers.empty()) {
            return false;
        }
        
        auto my_pos = get_position().point;
        float connection_range = 0.1f;  // Hitch points must be overlapping
        
        for (const auto& other_robot : all_robots) {
            // Skip self and non-slaves
            if (other_robot.get() == this || other_robot->role != RobotRole::SLAVE) {
                continue;
            }
            
            // Check if slave has front hitch
            if (!other_robot->info.hitches.count("front_hitch")) {
                continue;
            }
            
            // Calculate actual world positions of hitch points
            auto my_rear_hitch_pos = info.hitches["rear_hitch"];
            auto slave_front_hitch_pos = other_robot->info.hitches["front_hitch"];
            auto other_pos = other_robot->get_position().point;
            auto my_angle = get_position().angle.yaw;
            auto other_angle = other_robot->get_position().angle.yaw;
            
            // Transform hitch positions to world coordinates considering robot rotation
            float my_hitch_world_x = my_pos.x + my_rear_hitch_pos.bound.pose.point.x * cos(my_angle) - my_rear_hitch_pos.bound.pose.point.y * sin(my_angle);
            float my_hitch_world_y = my_pos.y + my_rear_hitch_pos.bound.pose.point.x * sin(my_angle) + my_rear_hitch_pos.bound.pose.point.y * cos(my_angle);
            
            float other_hitch_world_x = other_pos.x + slave_front_hitch_pos.bound.pose.point.x * cos(other_angle) - slave_front_hitch_pos.bound.pose.point.y * sin(other_angle);
            float other_hitch_world_y = other_pos.y + slave_front_hitch_pos.bound.pose.point.x * sin(other_angle) + slave_front_hitch_pos.bound.pose.point.y * cos(other_angle);
            
            // Check if hitch points are overlapping (very close)
            float hitch_dist = std::sqrt(std::pow(my_hitch_world_x - other_hitch_world_x, 2) + std::pow(my_hitch_world_y - other_hitch_world_y, 2));
            
            if (hitch_dist <= connection_range) {
                // Use the midpoint between the two hitch points as the joint position
                muli::Vec2 hitch_world_pos(
                    (my_hitch_world_x + other_hitch_world_x) / 2.0f,
                    (my_hitch_world_y + other_hitch_world_y) / 2.0f
                );
                
                auto new_joint = world->CreateRevoluteJoint(
                    chassis->get_body(),
                    other_robot->chassis->get_body(),
                    hitch_world_pos,
                    20.0f,  // frequency
                    0.8f,   // damping
                    10.0f   // joint mass
                );
                
                if (new_joint) {
                    // Add to followers using new system
                    connected_followers.push_back(other_robot.get());
                    connection_joints.push_back(new_joint);
                    other_robot->master_robot = this;
                    other_robot->role = RobotRole::FOLLOWER;  // Change slave to follower
                    spdlog::info("Connected {} to {}", info.name, other_robot->info.name);
                    return true;
                }
            }
        }
        
        return false;
    }

    void Robot::disconnect_trailer() {
        disconnect_all_followers();
    }

    void Robot::disconnect_all_followers() {
        for (size_t i = 0; i < connected_followers.size(); ++i) {
            Robot* follower = connected_followers[i];
            muli::RevoluteJoint* joint = connection_joints[i];
            
            if (joint) {
                world->Destroy(joint);
            }
            
            if (follower) {
                follower->role = RobotRole::SLAVE;  // Change back to slave
                follower->master_robot = nullptr;
                spdlog::info("Disconnected {} from {}", follower->info.name, info.name);
                
                // Recursively disconnect any sub-followers
                follower->disconnect_all_followers();
            }
        }
        
        connected_followers.clear();
        connection_joints.clear();
    }

    bool Robot::is_connected() const {
        return !connected_followers.empty() || master_robot != nullptr;
    }

    Robot* Robot::get_root_master() const {
        const Robot* current = this;
        while (current->master_robot != nullptr) {
            current = current->master_robot;
        }
        return const_cast<Robot*>(current);
    }

    // Spatial queries - delegate to simulator
    std::vector<Robot*> Robot::get_all_robots() const {
        if (!simulator) {
            return {};  // Return empty if no simulator reference
        }
        return simulator->get_all_robots();
    }

    Robot* Robot::get_closest_robot(float max_distance) const {
        if (!simulator) {
            return nullptr;  // Return null if no simulator reference
        }
        return simulator->get_closest_robot(*this, max_distance);
    }

    bool Robot::try_connect_nearby() {
        // Only MASTER robots or FOLLOWERS with available master hitches can initiate connections
        if ((role != RobotRole::MASTER && role != RobotRole::FOLLOWER) || !chassis) {
            return false;
        }
        
        // Check if this robot has any available master hitch
        bool has_available_master_hitch = false;
        for (const auto& hitch : chassis->hitches) {
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
        
        auto all_robots = get_all_robots();
        float connection_range = 2.0f;  // Increased range for easier connection
        
        for (Robot* other_robot : all_robots) {
            // Skip self and non-slaves
            if (other_robot == this || other_robot->role != RobotRole::SLAVE) {
                continue;
            }
            
            // Skip if other robot has no chassis or hitches
            if (!other_robot->chassis || other_robot->chassis->hitches.empty()) {
                continue;
            }
            
            // Try to connect my master hitches to their slave hitches
            for (const auto& my_hitch : chassis->hitches) {
                for (const auto& other_hitch : other_robot->chassis->hitches) {
                    // Only connect master hitch to slave hitch
                    if (!my_hitch.is_master || other_hitch.is_master) {
                        continue;
                    }
                    
                    // Get actual world positions of hitches (updated by tick)
                    concord::Point my_hitch_pos = my_hitch.pose.point;
                    concord::Point other_hitch_pos = other_hitch.pose.point;
                    
                    // Calculate distance between hitches
                    float hitch_dist = std::sqrt(
                        std::pow(my_hitch_pos.x - other_hitch_pos.x, 2) + 
                        std::pow(my_hitch_pos.y - other_hitch_pos.y, 2)
                    );
                    
                    if (hitch_dist <= connection_range) {
                        // Use the midpoint between the two hitch points as the joint position
                        muli::Vec2 hitch_world_pos(
                            (my_hitch_pos.x + other_hitch_pos.x) / 2.0f,
                            (my_hitch_pos.y + other_hitch_pos.y) / 2.0f
                        );
                        
                        auto new_joint = world->CreateRevoluteJoint(
                            chassis->get_body(),
                            other_robot->chassis->get_body(),
                            hitch_world_pos,
                            20.0f,  // frequency
                            0.8f,   // damping
                            10.0f   // joint mass
                        );
                        
                        if (new_joint) {
                            // Add to followers list
                            connected_followers.push_back(other_robot);
                            connection_joints.push_back(new_joint);
                            
                            // Set backward reference
                            other_robot->master_robot = this;
                            other_robot->role = RobotRole::FOLLOWER;
                            
                            spdlog::info("Connected {} to {} (hitch distance: {:.2f}m)", 
                                        info.name, other_robot->info.name, hitch_dist);
                            return true;
                        }
                    }
                }
            }
        }
        
        return false;
    }

} // namespace fs
