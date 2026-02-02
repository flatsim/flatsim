#include "flatsim/simulator/machine.hpp"
#include "flatsim/utils.hpp"
#include <cmath>
#include <concord/concord.hpp>
#include <cstdio>
#include <datapod/adapters.hpp>
#include <echo/echo.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace simulator {

    static float deg2rad(float deg) { return deg * (static_cast<float>(M_PI) / 180.0f); }

    static std::string get_prop(const datapod::Map<datapod::String, datapod::String> &props, const std::string &key) {
        auto it = props.find(datapod::String(key.c_str()));
        if (it == props.end()) {
            return "";
        }
        return std::string(it->second.c_str());
    }

    static bool has_prop(const datapod::Map<datapod::String, datapod::String> &props, const std::string &key) {
        return props.find(datapod::String(key.c_str())) != props.end();
    }

    void validate_model_for_flatsim(const datapod::robot::Model &model) {
        if (!has_prop(model.props, "flatsim.color.rgba")) {
            throw std::runtime_error("URDF missing robot prop: flatsim.color.rgba");
        }
        if (!has_prop(model.props, "flatsim.turning.radius")) {
            throw std::runtime_error("URDF missing robot prop: flatsim.turning.radius");
        }

        bool has_wheel = false;
        for (const auto &joint : model.joints) {
            if (has_prop(joint.props, "flatsim.side")) {
                has_wheel = true;
                if (!has_prop(joint.props, "flatsim.throttle_max")) {
                    throw std::runtime_error("URDF wheel joint missing prop: flatsim.throttle_max");
                }
                if (!has_prop(joint.props, "flatsim.throttle_diff")) {
                    throw std::runtime_error("URDF wheel joint missing prop: flatsim.throttle_diff");
                }
                if (!has_prop(joint.props, "flatsim.steering_max")) {
                    throw std::runtime_error("URDF wheel joint missing prop: flatsim.steering_max");
                }
                if (!has_prop(joint.props, "flatsim.steering_diff")) {
                    throw std::runtime_error("URDF wheel joint missing prop: flatsim.steering_diff");
                }
            }

            if (has_prop(joint.props, "flatsim.karosserie_name")) {
                if (!has_prop(joint.props, "flatsim.karosserie_sections")) {
                    throw std::runtime_error("URDF karosserie joint missing prop: flatsim.karosserie_sections");
                }
                if (!has_prop(joint.props, "flatsim.karosserie_has_physics")) {
                    throw std::runtime_error("URDF karosserie joint missing prop: flatsim.karosserie_has_physics");
                }
            }

            if (has_prop(joint.props, "flatsim.hitch_name") && !has_prop(joint.props, "flatsim.hitch_is_master")) {
                throw std::runtime_error("URDF hitch joint missing prop: flatsim.hitch_is_master");
            }

            if (has_prop(joint.props, "flatsim.tank_name")) {
                if (!has_prop(joint.props, "flatsim.tank_type")) {
                    throw std::runtime_error("URDF tank joint missing prop: flatsim.tank_type");
                }
                if (!has_prop(joint.props, "flatsim.tank_capacity")) {
                    throw std::runtime_error("URDF tank joint missing prop: flatsim.tank_capacity");
                }
            }

            if (has_prop(joint.props, "flatsim.power_name")) {
                if (!has_prop(joint.props, "flatsim.power_type")) {
                    throw std::runtime_error("URDF power joint missing prop: flatsim.power_type");
                }
                if (!has_prop(joint.props, "flatsim.power_capacity")) {
                    throw std::runtime_error("URDF power joint missing prop: flatsim.power_capacity");
                }
                if (!has_prop(joint.props, "flatsim.power_consumption_rate")) {
                    throw std::runtime_error("URDF power joint missing prop: flatsim.power_consumption_rate");
                }
                if (!has_prop(joint.props, "flatsim.power_charge_rate")) {
                    throw std::runtime_error("URDF power joint missing prop: flatsim.power_charge_rate");
                }
            }
        }

        if (!has_wheel) {
            throw std::runtime_error("URDF has no wheel joints (missing flatsim.side on any joint)");
        }
    }

    static datapod::Size get_geometry_size(const datapod::robot::Link &link) {
        if (!link.collisions.empty()) {
            const auto &geom = link.collisions[0].geom;
            if (geom.is_box()) {
                auto *b = geom.as_box();
                return b->size;
            }
            if (geom.is_cylinder()) {
                auto *cyl = geom.as_cylinder();
                return datapod::Size{cyl->length, cyl->radius * 2.0, cyl->radius * 2.0};
            }
        }
        if (!link.visuals.empty()) {
            const auto &geom = link.visuals[0].geom;
            if (geom.is_box()) {
                auto *b = geom.as_box();
                return b->size;
            }
            if (geom.is_cylinder()) {
                auto *cyl = geom.as_cylinder();
                return datapod::Size{cyl->length, cyl->radius * 2.0, cyl->radius * 2.0};
            }
        }
        return datapod::Size{0.1, 0.1, 0.1};
    }

    static bool str_eq(const datapod::String &s, const char *lit) { return std::string(s.c_str()) == std::string(lit); }

    static std::optional<types::LidarConfig> lidar_config_from_sensor_props(const datapod::robot::Sensor &sensor) {
        if (sensor.type.empty()) {
            return std::nullopt;
        }
        if (!str_eq(sensor.type, "lidar") && !str_eq(sensor.type, "LIDAR")) {
            return std::nullopt;
        }

        // Defaults: pick something visible/useful out of the box.
        // (Previously test_farmtrax set these explicitly.)
        types::LidarConfig cfg;
        cfg.enabled = true;
        cfg.min_range = 0.5f;
        cfg.max_range = 15.0f;
        cfg.fov_deg = 90.0f;
        cfg.resolution_deg = 2.0f;

        const auto get = [&](const char *key) { return get_prop(sensor.props, key); };

        if (auto v = get("sensor.flatsim.min_range"); !v.empty()) cfg.min_range = std::stof(v);
        if (auto v = get("sensor.flatsim.max_range"); !v.empty()) cfg.max_range = std::stof(v);
        if (auto v = get("sensor.flatsim.fov_deg"); !v.empty()) cfg.fov_deg = std::stof(v);
        if (auto v = get("sensor.flatsim.resolution_deg"); !v.empty()) cfg.resolution_deg = std::stof(v);
        if (auto v = get("sensor.flatsim.samples"); !v.empty()) {
            // If samples is given, derive resolution.
            const float samples = std::stof(v);
            if (samples > 1.0f) {
                cfg.resolution_deg = cfg.fov_deg / samples;
            }
        }

        return cfg;
    }

    types::Machine machine_from_model(const datapod::robot::Model &model, const datapod::Pose &spawn_pose,
                                      std::optional<pigment::RGB> color) {
        types::Machine machine;

        machine.name = "urdf";
        machine.uuid = "urdf";
        machine.type = machine.name;
        machine.rci = 0;

        {
            pigment::RGB parsed_color{128, 128, 128};
            auto rgba = get_prop(model.props, "flatsim.color.rgba");
            if (!rgba.empty()) {
                float r, g, b, a;
                if (std::sscanf(rgba.c_str(), "%f %f %f %f", &r, &g, &b, &a) >= 3) {
                    parsed_color = pigment::RGB(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
                }
            }
            machine.color = color.value_or(parsed_color);

            auto turning_radius = get_prop(model.props, "flatsim.turning.radius");
            if (!turning_radius.empty()) {
                machine.turning_radius = std::stof(turning_radius);
            }
        }

        // Sensors: pull LIDAR config from typed dp::robot::Link::sensor.
        for (const auto &link : model.links) {
            if (!link.sensor.has_value()) {
                continue;
            }
            auto cfg = lidar_config_from_sensor_props(link.sensor.value());
            if (cfg.has_value()) {
                machine.lidar = cfg;
                break;
            }
        }

        std::unordered_map<std::string, size_t> child_to_joint;
        for (size_t i = 0; i < model.joints.size(); ++i) {
            const auto &joint = model.joints[i];
            if (joint.child < model.links.size()) {
                child_to_joint[std::string(model.links[joint.child].name.c_str())] = i;
            }
        }

        struct SteeringInfo {
            float steering_max = 0.0f;
            float steering_diff = 0.0f;
            datapod::Pose origin;
        };
        std::unordered_map<std::string, SteeringInfo> steering_map;
        for (const auto &joint : model.joints) {
            if (joint.type != datapod::robot::Joint::Type::Revolute) {
                continue;
            }
            if (has_prop(joint.props, "flatsim.side")) {
                continue;
            }
            if (!has_prop(joint.props, "flatsim.steering_diff") && !has_prop(joint.props, "flatsim.steering_max")) {
                continue;
            }
            if (joint.child >= model.links.size()) {
                continue;
            }
            std::string child_name(model.links[joint.child].name.c_str());

            SteeringInfo info;
            info.origin = joint.origin;
            auto steering_diff = get_prop(joint.props, "flatsim.steering_diff");
            if (!steering_diff.empty()) {
                info.steering_diff = std::stof(steering_diff);
            }
            if (has_prop(joint.props, "flatsim.steering_max")) {
                info.steering_max = deg2rad(std::stof(get_prop(joint.props, "flatsim.steering_max")));
            } else if (joint.limits.has_value()) {
                info.steering_max = static_cast<float>(joint.limits->upper);
            }
            steering_map[child_name] = info;
        }

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto &joint : model.joints) {
            if (joint.child >= model.links.size()) {
                continue;
            }
            const auto &child_link = model.links[joint.child];
            std::string child_link_name(child_link.name.c_str());

            std::string parent_link_name;
            if (joint.parent < model.links.size()) {
                parent_link_name = std::string(model.links[joint.parent].name.c_str());
            }

            if (joint.type == datapod::robot::Joint::Type::Continuous && has_prop(joint.props, "flatsim.side")) {
                types::Wheel wheel;
                wheel.name = child_link_name;
                wheel.color = pigment::RGB(30, 30, 30);
                wheel.bound.size = get_geometry_size(child_link);

                datapod::Pose wheel_pose = joint.origin;
                auto steer_it = steering_map.find(parent_link_name);
                if (steer_it != steering_map.end()) {
                    wheel_pose.point.x += steer_it->second.origin.point.x;
                    wheel_pose.point.y += steer_it->second.origin.point.y;
                    wheel_pose.point.z += steer_it->second.origin.point.z;
                }
                wheel.bound.pose = wheel_pose;

                if (has_prop(joint.props, "flatsim.steering_max")) {
                    wheel.steering_max = deg2rad(std::stof(get_prop(joint.props, "flatsim.steering_max")));
                } else if (steer_it != steering_map.end()) {
                    wheel.steering_max = steer_it->second.steering_max;
                }
                machine.controls.steerings_max.push_back(wheel.steering_max);

                float steer_diff = 0.0f;
                auto steering_diff = get_prop(joint.props, "flatsim.steering_diff");
                if (!steering_diff.empty()) {
                    steer_diff = std::stof(steering_diff);
                } else if (steer_it != steering_map.end()) {
                    steer_diff = steer_it->second.steering_diff;
                }
                machine.controls.steerings_diff.push_back(deg2rad(steer_diff));

                wheel.throttle_max = std::stof(get_prop(joint.props, "flatsim.throttle_max"));
                machine.controls.throttles_max.push_back(wheel.throttle_max);
                machine.controls.throttles_diff.push_back(std::stof(get_prop(joint.props, "flatsim.throttle_diff")));
                machine.controls.left_side.push_back(get_prop(joint.props, "flatsim.side") == "left");

                machine.wheels.push_back(wheel);
                continue;
            }

            if (joint.type != datapod::robot::Joint::Type::Fixed) {
                continue;
            }

            if (has_prop(joint.props, "flatsim.karosserie_name")) {
                types::Karosserie kaross;
                kaross.name = get_prop(joint.props, "flatsim.karosserie_name");
                kaross.color = machine.color;
                kaross.has_physics = (get_prop(joint.props, "flatsim.karosserie_has_physics") == "true" ||
                                      get_prop(joint.props, "flatsim.karosserie_has_physics") == "1");
                kaross.bound.size = get_geometry_size(child_link);
                kaross.bound.pose = joint.origin;

                int sections = std::stoi(get_prop(joint.props, "flatsim.karosserie_sections"));
                for (int s = 0; s < sections; ++s) {
                    types::Section section;
                    section.name = "section_" + std::to_string(s);
                    section.bound.pose = utils::make_pose(0.0, 0.0, 0.0, 0.0);
                    section.bound.size = kaross.bound.size;
                    section.color = kaross.color;
                    kaross.sections.push_back(section);
                }
                machine.karosseries.push_back(kaross);

                if (kaross.name == "body") {
                    float x = static_cast<float>(joint.origin.point.x);
                    float y = static_cast<float>(joint.origin.point.y);
                    min_x = std::min(min_x, x - static_cast<float>(kaross.bound.size.x) / 2);
                    max_x = std::max(max_x, x + static_cast<float>(kaross.bound.size.x) / 2);
                    min_y = std::min(min_y, y - static_cast<float>(kaross.bound.size.y) / 2);
                    max_y = std::max(max_y, y + static_cast<float>(kaross.bound.size.y) / 2);
                }
            }

            if (has_prop(joint.props, "flatsim.hitch_name")) {
                types::Hitch hitch;
                hitch.name = get_prop(joint.props, "flatsim.hitch_name");
                hitch.is_master = (get_prop(joint.props, "flatsim.hitch_is_master") == "true" ||
                                   get_prop(joint.props, "flatsim.hitch_is_master") == "1");
                hitch.color = pigment::RGB(50, 50, 50);
                hitch.bound.size = get_geometry_size(child_link);
                hitch.bound.pose = joint.origin;
                machine.hitches[hitch.name] = hitch;
            }

            if (has_prop(joint.props, "flatsim.tank_name")) {
                types::Tank tank;
                tank.name = get_prop(joint.props, "flatsim.tank_name");
                tank.capacity = std::stof(get_prop(joint.props, "flatsim.tank_capacity"));
                tank.type = (get_prop(joint.props, "flatsim.tank_type") == "WASTE") ? types::ContainerType::WASTE
                                                                                    : types::ContainerType::HARVEST;
                tank.bound.size = get_geometry_size(child_link);
                tank.bound.pose = joint.origin;
                machine.tank = tank;
            }

            if (has_prop(joint.props, "flatsim.power_name")) {
                types::Power power;
                power.name = get_prop(joint.props, "flatsim.power_name");
                power.capacity = std::stof(get_prop(joint.props, "flatsim.power_capacity"));
                power.consumption_rate = std::stof(get_prop(joint.props, "flatsim.power_consumption_rate"));
                power.charge_rate = std::stof(get_prop(joint.props, "flatsim.power_charge_rate"));
                power.type = (get_prop(joint.props, "flatsim.power_type") == "BATTERY") ? types::PowerType::BATTERY
                                                                                        : types::PowerType::FUEL;
                machine.power_source = power;
            }
        }

        float width = max_x - min_x;
        float height = max_y - min_y;
        if (!std::isfinite(width) || !std::isfinite(height) || width <= 0.0f || height <= 0.0f) {
            width = 1.0f;
            height = 1.0f;
        }
        machine.bound.pose = spawn_pose;
        machine.bound.size = datapod::Size{width, height, 0.3};

        bool has_slave_hitch = false;
        for (const auto &[name, hitch] : machine.hitches) {
            (void)name;
            if (!hitch.is_master) {
                has_slave_hitch = true;
                break;
            }
        }
        if (has_slave_hitch && machine.wheels.empty()) {
            machine.role = types::MachineRole::SLAVE;
            machine.slave = true;
        } else if (has_slave_hitch) {
            machine.role = types::MachineRole::MASTER;
        } else {
            machine.role = types::MachineRole::MASTER;
        }

        return machine;
    }

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<flywheel::World> world,
                     const types::Machine &config, uint32_t group)
        : rec_(rec), world_(world), config_(config) {
        // Create collision filter (use bit/mask system)
        filter_.group = 0;
        filter_.bit = 1 << group;
        filter_.mask = ~(1 << group);
    }

    void Machine::create() {
        if (!world_) {
            std::cerr << "[Simulator] Cannot create machine - missing world" << std::endl;
            return;
        }

        // Create chassis which manages all physics
        chassis_ = std::make_unique<Chassis>(world_, rec_, filter_, &config_, &state_);
        chassis_->init(config_);
    }

    void Machine::destroy() {
        // Clear Rerun visualization for this machine's namespace recursively
        if (rec_) {
            rec_->log(config_.uuid, rerun::Clear::RECURSIVE);
            rec_->log_with_static(config_.uuid, true, rerun::Clear::RECURSIVE);
        }

        if (chassis_) {
            chassis_->destroy();
        }
        chassis_.reset();
    }

    void Machine::apply_control(const types::WheelControl &control, float dt) {
        if (!chassis_) return;

        // Apply brake if requested
        if (control.brake > 0.0f) {
            chassis_->brake(control.brake);
            return;
        }

        // Apply per-wheel steering and throttle
        chassis_->update(control.steering, control.throttle, dt);
    }

    void Machine::tick(float dt) {
        if (!chassis_) return;

        // Update pose from physics
        config_.bound.pose = chassis_->get_pose();

        // Tick chassis (updates wheels, karosseries, hitches)
        chassis_->tick(dt);
    }

    void Machine::tock(datapod::Geo datum) {
        if (!chassis_) return;
        if (!rec_) return;

        // Create label with role info
        std::string role_prefix;
        switch (config_.role) {
        case types::MachineRole::MASTER:
            role_prefix = "(M)";
            break;
        case types::MachineRole::FOLLOWER:
            role_prefix = "(F)";
            break;
        case types::MachineRole::SLAVE:
            role_prefix = "(S)";
            break;
        }
        std::string label = role_prefix + config_.uuid;

        chassis_->tock(label);

        // GPS coordinates visualization - use current body position
        if (chassis_->body) {
            auto x = chassis_->body->GetPosition().x;
            auto y = chassis_->body->GetPosition().y;
            datapod::Point current_pos{x, y, 0.0};
            concord::frame::ENU enu{current_pos, datum};
            auto wgs_coords = concord::frame::to_wgs(enu);

            rec_->log_static(config_.uuid + "/gps",
                             rerun::GeoPoints({{wgs_coords.latitude, wgs_coords.longitude}})
                                 .with_colors({rerun::Color(config_.color.r(), config_.color.g(), config_.color.b())}));
        }
    }

    types::ser::MachineState Machine::get_state() const {
        types::ser::MachineState ms;
        ms.uuid = datapod::String(config_.uuid);

        if (chassis_ && chassis_->body) {
            ms.pose.position.x = chassis_->body->GetPosition().x;
            ms.pose.position.y = chassis_->body->GetPosition().y;
            // Add M_PI/2 to convert from body angle to world heading
            // (model forward is +Y, so body angle 0 = heading +90deg)
            ms.pose.angle = chassis_->body->GetAngle() + M_PI / 2;
            ms.velocity.x = chassis_->body->GetLinearVelocity().x;
            ms.velocity.y = chassis_->body->GetLinearVelocity().y;
            ms.angular_vel = chassis_->body->GetAngularVelocity();
        }

        return ms;
    }

    Hitch *Machine::find_hitch(const std::string &name) {
        if (!chassis_) return nullptr;

        for (auto &hitch : chassis_->hitches) {
            if (hitch.name == name) {
                return &hitch;
            }
        }
        return nullptr;
    }

    void Machine::teleport(const datapod::Pose &pose) {
        if (chassis_) {
            chassis_->teleport(pose);
        }
    }

    void Machine::brake(float brake_force) {
        if (chassis_) {
            chassis_->brake(brake_force);
        }
    }

    void Machine::update_color(const pigment::RGB &new_color) {
        config_.color = new_color;
        if (chassis_) {
            chassis_->update_color(new_color);
        }
    }

    void Machine::update_sensors(Data &data, const datapod::Geo &datum, float dt) {
        if (!chassis_ || !chassis_->body) return;

        // Get current state
        auto pose = chassis_->get_pose();

        // IMPORTANT: The physics body's angle 0 = +X direction, but the model's forward is +Y.
        // So we add M_PI/2 to convert body angle to heading (same as get_state() does for the agent).
        float heading = utils::get_yaw(pose) + M_PI / 2.0f;

        auto vel = chassis_->body->GetLinearVelocity();
        float linear_vel = vel.x * std::cos(heading) + vel.y * std::sin(heading);
        float angular_vel = chassis_->body->GetAngularVelocity();

        // GPS - always fill if datum is set
        if (datum.is_set()) {
            // Use corrected heading for GPS
            datapod::Pose gps_pose = pose;
            utils::set_yaw(gps_pose, heading);
            sensor_data_.gps = data.pose_to_gps(gps_pose, std::abs(linear_vel));
            sensor_data_.has_gps = true;
        }

        // IMU - always fill
        sensor_data_.imu = data.compute_imu(pose, linear_vel, angular_vel, prev_linear_vel_, dt);
        sensor_data_.has_imu = true;
        prev_linear_vel_ = linear_vel;

        // LIDAR - only if configured
        if (config_.lidar.has_value() && config_.lidar->enabled) {
            const auto &lidar_cfg = config_.lidar.value();
            // Use corrected heading for LIDAR scan direction
            datapod::Pose lidar_pose = pose;
            utils::set_yaw(lidar_pose, heading);
            sensor_data_.lidar = data.scan_lidar(lidar_pose, lidar_cfg.min_range, lidar_cfg.max_range,
                                                 lidar_cfg.fov_deg, lidar_cfg.resolution_deg, filter_);
            sensor_data_.has_lidar = true;
        }
    }

} // namespace simulator
