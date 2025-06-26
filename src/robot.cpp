#include "flatsim/robot.hpp"
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
        // Create label with power percentage
        std::string label = info.name;
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
        
        // Already connected
        if (connected_slave) {
            return false;
        }
        
        auto my_pos = get_position().point;
        float connection_range = 5.0f;  // 5 unit connection range
        
        for (const auto& other_robot : all_robots) {
            // Skip self and non-slaves
            if (other_robot.get() == this || other_robot->role != RobotRole::SLAVE) {
                continue;
            }
            
            // Check if slave has front hitch
            if (!other_robot->info.hitches.count("front_hitch")) {
                continue;
            }
            
            // Check distance
            auto other_pos = other_robot->get_position().point;
            float dist = std::sqrt(std::pow(my_pos.x - other_pos.x, 2) + std::pow(my_pos.y - other_pos.y, 2));
            
            if (dist <= connection_range) {
                // Create joint between rear hitch and front hitch
                auto my_rear_hitch_pos = info.hitches["rear_hitch"];
                auto slave_front_hitch_pos = other_robot->info.hitches["front_hitch"];
                
                // Calculate world positions of hitches
                muli::Vec2 hitch_world_pos(
                    my_pos.x + my_rear_hitch_pos.pose.point.x,
                    my_pos.y + my_rear_hitch_pos.pose.point.y
                );
                
                connection_joint = world->CreateRevoluteJoint(
                    chassis->get_body(),
                    other_robot->chassis->get_body(),
                    hitch_world_pos,
                    20.0f,  // frequency
                    0.8f,   // damping
                    10.0f   // joint mass
                );
                
                if (connection_joint) {
                    connected_slave = other_robot.get();
                    other_robot->role = RobotRole::FOLLOWER;  // Change slave to follower
                    spdlog::info("Connected {} to {}", info.name, other_robot->info.name);
                    return true;
                }
            }
        }
        
        return false;
    }

    void Robot::disconnect_trailer() {
        if (connection_joint && connected_slave) {
            world->Destroy(connection_joint);
            connection_joint = nullptr;
            connected_slave->role = RobotRole::SLAVE;  // Change back to slave
            spdlog::info("Disconnected trailer from {}", info.name);
            connected_slave = nullptr;
        }
    }

    bool Robot::is_connected() const {
        return connected_slave != nullptr && connection_joint != nullptr;
    }

} // namespace fs
