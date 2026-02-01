#include "flatsim/agent/loader/loader.hpp"
#include "flatsim/utils.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <regex>
#include <sstream>

#include <agent47/model/urdf.hpp>

namespace agent {

    static float deg2rad(float deg) { return deg * (M_PI / 180.0f); }

    // Helper to get a prop value, returning empty string if not found
    static std::string get_prop(const dp::Map<dp::String, dp::String> &props, const std::string &key) {
        auto it = props.find(dp::String(key.c_str()));
        if (it != props.end()) {
            return std::string(it->second.c_str());
        }
        return "";
    }

    // Helper to check if a prop exists
    static bool has_prop(const dp::Map<dp::String, dp::String> &props, const std::string &key) {
        return props.find(dp::String(key.c_str())) != props.end();
    }

    std::string Loader::generate_uuid() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 15);
        static std::uniform_int_distribution<> dis2(8, 11);

        std::stringstream ss;
        ss << std::hex;
        for (int i = 0; i < 8; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (int i = 0; i < 4; i++) {
            ss << dis(gen);
        }
        ss << "-4";
        for (int i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        ss << dis2(gen);
        for (int i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (int i = 0; i < 12; i++) {
            ss << dis(gen);
        }
        return ss.str();
    }

    std::vector<std::filesystem::path> Loader::find_machine_files(const std::filesystem::path &directory) {
        std::vector<std::filesystem::path> machine_files;

        if (!std::filesystem::exists(directory)) {
            return machine_files;
        }

        for (const auto &entry : std::filesystem::directory_iterator(directory)) {
            if (entry.is_regular_file()) {
                auto ext = entry.path().extension();
                if (ext == ".urdf") {
                    machine_files.push_back(entry.path());
                }
            }
        }

        return machine_files;
    }

    // ========================================================================
    // URDF Loading Implementation (using props maps)
    // ========================================================================

    // Parse robot-level props: flatsim.color.rgba, flatsim.turning.radius
    struct RobotFlatsimExt {
        pigment::RGB color{128, 128, 128};
        float turning_radius = 1.0f;
        bool has_color = false;
    };

    static RobotFlatsimExt parse_robot_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        RobotFlatsimExt ext;

        // Parse flatsim.color.rgba = "r g b a"
        std::string rgba = get_prop(props, "flatsim.color.rgba");
        if (!rgba.empty()) {
            float r, g, b, a;
            if (sscanf(rgba.c_str(), "%f %f %f %f", &r, &g, &b, &a) >= 3) {
                ext.color = pigment::RGB(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
                ext.has_color = true;
            }
        }

        // Parse flatsim.turning.radius = "..."
        std::string radius_str = get_prop(props, "flatsim.turning.radius");
        if (!radius_str.empty()) {
            ext.turning_radius = std::stof(radius_str);
        }

        return ext;
    }

    // Wheel extension from joint.props: flatsim.side, flatsim.throttle_max, etc.
    struct WheelFlatsimExt {
        std::string side; // "left" or "right"
        float throttle_max = 1.0f;
        float throttle_diff = 0.0f;
        std::optional<float> steering_max; // degrees, can be negative for opposite direction
        float steering_diff = 0.0f;
    };

    static std::optional<WheelFlatsimExt> parse_wheel_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        // Wheel joints have "flatsim.side" attribute
        if (!has_prop(props, "flatsim.side")) {
            return std::nullopt;
        }

        WheelFlatsimExt ext;
        ext.side = get_prop(props, "flatsim.side");

        std::string throttle_max = get_prop(props, "flatsim.throttle_max");
        if (!throttle_max.empty()) {
            ext.throttle_max = std::stof(throttle_max);
        }

        std::string throttle_diff = get_prop(props, "flatsim.throttle_diff");
        if (!throttle_diff.empty()) {
            ext.throttle_diff = std::stof(throttle_diff);
        }

        std::string steering_max = get_prop(props, "flatsim.steering_max");
        if (!steering_max.empty()) {
            ext.steering_max = std::stof(steering_max);
        }

        std::string steering_diff = get_prop(props, "flatsim.steering_diff");
        if (!steering_diff.empty()) {
            ext.steering_diff = std::stof(steering_diff);
        }

        return ext;
    }

    // Steering extension from joint.props: flatsim.steering_diff, flatsim.steering_max (no side attr)
    struct SteeringFlatsimExt {
        float steering_diff = 0.0f;
        std::optional<float> steering_max; // degrees, negative = opposite direction
    };

    static std::optional<SteeringFlatsimExt> parse_steering_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        // Steering joints have steering_diff but NOT side
        if (has_prop(props, "flatsim.side")) {
            return std::nullopt;
        }
        if (!has_prop(props, "flatsim.steering_diff") && !has_prop(props, "flatsim.steering_max")) {
            return std::nullopt;
        }

        SteeringFlatsimExt ext;

        std::string steering_diff = get_prop(props, "flatsim.steering_diff");
        if (!steering_diff.empty()) {
            ext.steering_diff = std::stof(steering_diff);
        }

        std::string steering_max = get_prop(props, "flatsim.steering_max");
        if (!steering_max.empty()) {
            ext.steering_max = std::stof(steering_max);
        }

        return ext;
    }

    // Karosserie extension: flatsim.karosserie_name, flatsim.karosserie_sections, flatsim.karosserie_has_physics
    struct KarosserieFlatsimExt {
        std::string name;
        int sections = 0;
        bool has_physics = true;
    };

    static std::optional<KarosserieFlatsimExt>
    parse_karosserie_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        if (!has_prop(props, "flatsim.karosserie_name")) {
            return std::nullopt;
        }

        KarosserieFlatsimExt ext;
        ext.name = get_prop(props, "flatsim.karosserie_name");

        std::string sections = get_prop(props, "flatsim.karosserie_sections");
        if (!sections.empty()) {
            ext.sections = std::stoi(sections);
        }

        std::string has_physics = get_prop(props, "flatsim.karosserie_has_physics");
        if (!has_physics.empty()) {
            ext.has_physics = (has_physics == "true" || has_physics == "1");
        }

        return ext;
    }

    // Hitch extension: flatsim.hitch_name, flatsim.hitch_is_master
    struct HitchFlatsimExt {
        std::string name;
        bool is_master = true;
    };

    static std::optional<HitchFlatsimExt> parse_hitch_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        if (!has_prop(props, "flatsim.hitch_name")) {
            return std::nullopt;
        }

        HitchFlatsimExt ext;
        ext.name = get_prop(props, "flatsim.hitch_name");

        std::string is_master = get_prop(props, "flatsim.hitch_is_master");
        if (!is_master.empty()) {
            ext.is_master = (is_master == "true" || is_master == "1");
        }

        return ext;
    }

    // Tank extension: flatsim.tank_name, flatsim.tank_type, flatsim.tank_capacity
    struct TankFlatsimExt {
        std::string name;
        std::string type = "HARVEST";
        float capacity = 1000.0f;
    };

    static std::optional<TankFlatsimExt> parse_tank_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        if (!has_prop(props, "flatsim.tank_name")) {
            return std::nullopt;
        }

        TankFlatsimExt ext;
        ext.name = get_prop(props, "flatsim.tank_name");

        std::string type = get_prop(props, "flatsim.tank_type");
        if (!type.empty()) {
            ext.type = type;
        }

        std::string capacity = get_prop(props, "flatsim.tank_capacity");
        if (!capacity.empty()) {
            ext.capacity = std::stof(capacity);
        }

        return ext;
    }

    // Power extension: flatsim.power_name, flatsim.power_type, flatsim.power_capacity, etc.
    struct PowerFlatsimExt {
        std::string name;
        std::string type = "FUEL";
        float capacity = 100.0f;
        float consumption_rate = 0.01f;
        float charge_rate = 0.0f;
    };

    static std::optional<PowerFlatsimExt> parse_power_flatsim_ext(const dp::Map<dp::String, dp::String> &props) {
        if (!has_prop(props, "flatsim.power_name")) {
            return std::nullopt;
        }

        PowerFlatsimExt ext;
        ext.name = get_prop(props, "flatsim.power_name");

        std::string type = get_prop(props, "flatsim.power_type");
        if (!type.empty()) {
            ext.type = type;
        }

        std::string capacity = get_prop(props, "flatsim.power_capacity");
        if (!capacity.empty()) {
            ext.capacity = std::stof(capacity);
        }

        std::string consumption_rate = get_prop(props, "flatsim.power_consumption_rate");
        if (!consumption_rate.empty()) {
            ext.consumption_rate = std::stof(consumption_rate);
        }

        std::string charge_rate = get_prop(props, "flatsim.power_charge_rate");
        if (!charge_rate.empty()) {
            ext.charge_rate = std::stof(charge_rate);
        }

        return ext;
    }

    // Extract geometry size from link's visual/collision
    static datapod::Size get_geometry_size(const datapod::robot::Link &link) {
        // Prefer collision, fall back to visual
        if (!link.collisions.empty()) {
            const auto &geom = link.collisions[0].geom;
            if (geom.is_box()) {
                return geom.as_box()->size;
            }
            if (geom.is_cylinder()) {
                // For wheel: height = radius*2, width = length
                auto *cyl = geom.as_cylinder();
                return datapod::Size{cyl->length, cyl->radius * 2.0, cyl->radius * 2.0};
            }
        }
        if (!link.visuals.empty()) {
            const auto &geom = link.visuals[0].geom;
            if (geom.is_box()) {
                return geom.as_box()->size;
            }
            if (geom.is_cylinder()) {
                auto *cyl = geom.as_cylinder();
                return datapod::Size{cyl->length, cyl->radius * 2.0, cyl->radius * 2.0};
            }
        }
        return datapod::Size{0.1, 0.1, 0.1};
    }

    types::Machine Loader::load_from_urdf(const std::filesystem::path &urdf_path, datapod::Pose spawn_pose,
                                          std::optional<pigment::RGB> color) {
        // Read URDF file
        std::ifstream file(urdf_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open URDF file: " + urdf_path.string());
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string xml_str = buffer.str();
        file.close();

        // Parse URDF using robomod
        auto result = robomod::from_urdf_string(dp::String(xml_str.c_str()));
        if (result.is_err()) {
            throw std::runtime_error("Failed to parse URDF: " + urdf_path.string());
        }
        datapod::robot::Model model = result.value();

        types::Machine machine;

        // Extract robot name from filename
        machine.name = urdf_path.stem().string();
        machine.uuid = generate_uuid();
        machine.type = machine.name;
        machine.rci = 0;

        // Parse robot-level props (color, turning radius)
        RobotFlatsimExt robot_ext = parse_robot_flatsim_ext(model.props);
        machine.color = color.value_or(robot_ext.color);
        machine.turning_radius = robot_ext.turning_radius;

        // Build link name -> index map
        std::unordered_map<std::string, size_t> link_map;
        for (size_t i = 0; i < model.links.size(); ++i) {
            link_map[std::string(model.links[i].name.c_str())] = i;
        }

        // Build joint name -> index map and child_link -> joint map
        std::unordered_map<std::string, size_t> joint_map;
        std::unordered_map<std::string, size_t> child_to_joint;
        for (size_t i = 0; i < model.joints.size(); ++i) {
            const auto &joint = model.joints[i];
            joint_map[std::string(joint.name.c_str())] = i;
            // Find child link name
            if (joint.child < model.links.size()) {
                std::string child_name(model.links[joint.child].name.c_str());
                child_to_joint[child_name] = i;
            }
        }

        // Track steering joints for wheel processing
        // Map: steer_link_name -> {steering_max, steering_diff, steer_joint_origin}
        struct SteeringInfo {
            float steering_max = 0.0f;
            float steering_diff = 0.0f;
            datapod::Pose origin;
        };
        std::unordered_map<std::string, SteeringInfo> steering_map;

        // Compute machine bounding box
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        // First pass: collect steering joints (revolute with steering_diff but no side)
        for (size_t i = 0; i < model.joints.size(); ++i) {
            const auto &joint = model.joints[i];

            if (joint.type == datapod::robot::Joint::Type::Revolute) {
                auto steer_ext = parse_steering_flatsim_ext(joint.props);
                if (steer_ext) {
                    // Get child link name (the steer_link)
                    if (joint.child < model.links.size()) {
                        std::string child_name(model.links[joint.child].name.c_str());
                        SteeringInfo info;
                        info.steering_diff = steer_ext->steering_diff;
                        // Use steering_max from extension if provided (can be negative for opposite direction)
                        // Otherwise fall back to joint limit upper bound
                        if (steer_ext->steering_max.has_value()) {
                            info.steering_max = deg2rad(steer_ext->steering_max.value());
                        } else if (joint.limits.has_value()) {
                            info.steering_max = static_cast<float>(joint.limits->upper);
                        }
                        info.origin = joint.origin;
                        steering_map[child_name] = info;
                    }
                }
            }
        }

        // Second pass: process all joints
        for (size_t i = 0; i < model.joints.size(); ++i) {
            const auto &joint = model.joints[i];

            // Get child link
            if (joint.child >= model.links.size()) {
                continue;
            }
            const auto &child_link = model.links[joint.child];
            std::string child_link_name(child_link.name.c_str());

            // Get parent link name for steering lookup
            std::string parent_link_name;
            if (joint.parent < model.links.size()) {
                parent_link_name = std::string(model.links[joint.parent].name.c_str());
            }

            // Check for wheel (continuous joint with "side" attribute)
            if (joint.type == datapod::robot::Joint::Type::Continuous) {
                auto wheel_ext = parse_wheel_flatsim_ext(joint.props);
                if (wheel_ext) {
                    types::Wheel wheel;
                    wheel.name = child_link_name;
                    wheel.color = pigment::RGB(30, 30, 30);

                    // Get wheel geometry
                    datapod::Size size = get_geometry_size(child_link);
                    wheel.bound.size = size;

                    // Compute wheel position
                    datapod::Pose wheel_pose = joint.origin;

                    // Check if parent is a steering link
                    auto steer_it = steering_map.find(parent_link_name);
                    if (steer_it != steering_map.end()) {
                        // Combine steer joint origin + wheel joint origin
                        wheel_pose.point.x += steer_it->second.origin.point.x;
                        wheel_pose.point.y += steer_it->second.origin.point.y;
                        wheel_pose.point.z += steer_it->second.origin.point.z;
                    }

                    // Determine steering_max: wheel ext takes priority, then steering joint, then 0
                    if (wheel_ext->steering_max.has_value()) {
                        wheel.steering_max = deg2rad(wheel_ext->steering_max.value());
                    } else if (steer_it != steering_map.end()) {
                        wheel.steering_max = steer_it->second.steering_max;
                    } else {
                        wheel.steering_max = 0.0f;
                    }
                    machine.controls.steerings_max.push_back(wheel.steering_max);

                    // Determine steering_diff: wheel ext takes priority, then steering joint
                    float steer_diff = wheel_ext->steering_diff;
                    if (steer_diff == 0.0f && steer_it != steering_map.end()) {
                        steer_diff = steer_it->second.steering_diff;
                    }
                    machine.controls.steerings_diff.push_back(deg2rad(steer_diff));

                    wheel.bound.pose = wheel_pose;
                    wheel.throttle_max = wheel_ext->throttle_max;

                    machine.wheels.push_back(wheel);
                    machine.controls.throttles_max.push_back(wheel_ext->throttle_max);
                    machine.controls.throttles_diff.push_back(wheel_ext->throttle_diff);
                    machine.controls.left_side.push_back(wheel_ext->side == "left");
                }
            }

            // Check for karosserie (fixed joint with karosserie_name)
            if (joint.type == datapod::robot::Joint::Type::Fixed) {
                auto karo_ext = parse_karosserie_flatsim_ext(joint.props);
                if (karo_ext) {
                    types::Karosserie kaross;
                    kaross.name = karo_ext->name;
                    kaross.color = machine.color;
                    kaross.has_physics = karo_ext->has_physics;

                    datapod::Size size = get_geometry_size(child_link);
                    kaross.bound.size = size;
                    kaross.bound.pose = joint.origin;

                    // Create sections if specified
                    for (int s = 0; s < karo_ext->sections; ++s) {
                        types::Section section;
                        section.name = "section_" + std::to_string(s);
                        section.bound.pose = utils::make_pose(0.0, 0.0, 0.0, 0.0);
                        section.bound.size = kaross.bound.size;
                        section.color = kaross.color;
                        kaross.sections.push_back(section);
                    }

                    machine.karosseries.push_back(kaross);

                    // Update bounds only from "body" karosserie
                    if (karo_ext->name == "body") {
                        float x = static_cast<float>(joint.origin.point.x);
                        float y = static_cast<float>(joint.origin.point.y);
                        min_x = std::min(min_x, x - static_cast<float>(size.x) / 2);
                        max_x = std::max(max_x, x + static_cast<float>(size.x) / 2);
                        min_y = std::min(min_y, y - static_cast<float>(size.y) / 2);
                        max_y = std::max(max_y, y + static_cast<float>(size.y) / 2);
                    }
                }

                // Check for hitch
                auto hitch_ext = parse_hitch_flatsim_ext(joint.props);
                if (hitch_ext) {
                    types::Hitch hitch;
                    hitch.name = hitch_ext->name;
                    hitch.is_master = hitch_ext->is_master;
                    hitch.color = pigment::RGB(50, 50, 50);

                    datapod::Size size = get_geometry_size(child_link);
                    hitch.bound.size = size;
                    hitch.bound.pose = joint.origin;

                    machine.hitches[hitch.name] = hitch;
                }

                // Check for tank
                auto tank_ext = parse_tank_flatsim_ext(joint.props);
                if (tank_ext) {
                    types::Tank tank;
                    tank.name = tank_ext->name;
                    tank.capacity = tank_ext->capacity;
                    tank.type =
                        (tank_ext->type == "WASTE") ? types::ContainerType::WASTE : types::ContainerType::HARVEST;

                    datapod::Size size = get_geometry_size(child_link);
                    tank.bound.size = size;
                    tank.bound.pose = joint.origin;

                    machine.tank = tank;
                }

                // Check for power
                auto power_ext = parse_power_flatsim_ext(joint.props);
                if (power_ext) {
                    types::Power power;
                    power.name = power_ext->name;
                    power.capacity = power_ext->capacity;
                    power.consumption_rate = power_ext->consumption_rate;
                    power.charge_rate = power_ext->charge_rate;
                    power.type = (power_ext->type == "BATTERY") ? types::PowerType::BATTERY : types::PowerType::FUEL;

                    machine.power_source = power;
                }
            }
        }

        // Set machine bounding box
        float width = max_x - min_x;
        float height = max_y - min_y;
        machine.bound.pose = spawn_pose;
        machine.bound.size = datapod::Size{width, height, 0.3};

        // Determine role based on presence of hitches
        bool has_slave_hitch = false;
        for (const auto &[name, hitch] : machine.hitches) {
            if (!hitch.is_master) {
                has_slave_hitch = true;
                break;
            }
        }
        if (has_slave_hitch && machine.wheels.empty()) {
            machine.role = types::MachineRole::SLAVE;
            machine.slave = true;
        } else if (has_slave_hitch) {
            // Has both master and slave hitches - could be either
            machine.role = types::MachineRole::MASTER;
        } else {
            machine.role = types::MachineRole::MASTER;
        }

        return machine;
    }

} // namespace agent
