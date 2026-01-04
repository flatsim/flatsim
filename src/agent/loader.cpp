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
#include <sstream>

namespace agent {

    static float deg2rad(float deg) { return deg * (M_PI / 180.0f); }

    // RAII wrapper for json_value_s to ensure proper cleanup
    struct JsonDeleter {
        void operator()(json_value_s *ptr) const {
            if (ptr) free(ptr);
        }
    };
    using JsonPtr = std::unique_ptr<json_value_s, JsonDeleter>;

    // Helper functions implementation
    json_object_element_s *Loader::find_element(json_object_s *obj, const char *key) {
        if (!obj) return nullptr;
        for (auto *elem = obj->start; elem; elem = elem->next) {
            if (elem->name && strcmp(elem->name->string, key) == 0) {
                return elem;
            }
        }
        return nullptr;
    }

    std::string Loader::get_string(json_value_s *val) {
        if (!val || val->type != json_type_string) return "";
        auto *str = static_cast<json_string_s *>(val->payload);
        return std::string(str->string, str->string_size);
    }

    double Loader::get_number(json_value_s *val) {
        if (!val || val->type != json_type_number) return 0.0;
        auto *num = static_cast<json_number_s *>(val->payload);
        return std::stod(std::string(num->number, num->number_size));
    }

    int Loader::get_int(json_value_s *val) {
        if (!val || val->type != json_type_number) return 0;
        auto *num = static_cast<json_number_s *>(val->payload);
        return std::stoi(std::string(num->number, num->number_size));
    }

    bool Loader::get_bool(json_value_s *val) {
        if (!val) return false;
        return val->type == json_type_true;
    }

    json_object_s *Loader::get_object(json_value_s *val) {
        if (!val || val->type != json_type_object) return nullptr;
        return static_cast<json_object_s *>(val->payload);
    }

    json_array_s *Loader::get_array(json_value_s *val) {
        if (!val || val->type != json_type_array) return nullptr;
        return static_cast<json_array_s *>(val->payload);
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

    types::Machine Loader::load_from_json(const std::filesystem::path &json_path, datapod::Pose spawn_pose,
                                          std::optional<pigment::RGB> color) {

        std::ifstream file(json_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open machine file: " + json_path.string());
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string json_str = buffer.str();
        file.close();

        json_value_s *root = json_parse(json_str.c_str(), json_str.size());
        if (!root) {
            throw std::runtime_error("Failed to parse JSON file: " + json_path.string());
        }
        JsonPtr jv(root);

        json_object_s *j = get_object(jv.get());
        if (!j) {
            throw std::runtime_error("JSON root is not an object");
        }

        types::Machine machine;

        // Parse info section
        auto *info_elem = find_element(j, "info");
        if (!info_elem) {
            throw std::runtime_error("Missing 'info' field in JSON");
        }
        json_object_s *info = get_object(info_elem->value);
        if (!info) {
            throw std::runtime_error("'info' field is not an object");
        }

        auto *type_elem = find_element(info, "type");
        if (type_elem) {
            machine.type = get_string(type_elem->value);
        }

        auto *name_elem = find_element(info, "name");
        if (name_elem) {
            machine.name = get_string(name_elem->value);
        }

        auto *uuid_elem = find_element(info, "uuid");
        std::string uuid_str = uuid_elem ? get_string(uuid_elem->value) : "";
        machine.uuid = (uuid_str.empty() || uuid_str == "") ? generate_uuid() : uuid_str;

        auto *rci_elem = find_element(info, "rci");
        if (rci_elem) {
            machine.rci = static_cast<uint32_t>(get_int(rci_elem->value));
        }

        auto *works_on_elem = find_element(info, "works_on");
        if (works_on_elem) {
            json_array_s *works_on = get_array(works_on_elem->value);
            if (works_on) {
                for (auto *elem = works_on->start; elem; elem = elem->next) {
                    machine.works_on.push_back(get_string(elem->value));
                }
            }
        }

        auto *role_elem = find_element(info, "role");
        std::string role_str = role_elem ? get_string(role_elem->value) : "MASTER";
        if (role_str == "SLAVE") {
            machine.role = types::MachineRole::SLAVE;
        } else if (role_str == "FOLLOWER") {
            machine.role = types::MachineRole::FOLLOWER;
        } else {
            machine.role = types::MachineRole::MASTER;
        }

        // Parse dimensions
        auto *dims_elem = find_element(j, "dimensions");
        if (!dims_elem) {
            throw std::runtime_error("Missing 'dimensions' field in JSON");
        }
        json_object_s *dims = get_object(dims_elem->value);
        if (!dims) {
            throw std::runtime_error("'dimensions' field is not an object");
        }

        auto *width_elem = find_element(dims, "width");
        auto *height_elem = find_element(dims, "height");
        float width = width_elem ? static_cast<float>(get_number(width_elem->value)) : 0.0f;
        float height = height_elem ? static_cast<float>(get_number(height_elem->value)) : 0.0f;
        machine.bound.pose = spawn_pose;
        machine.bound.size = datapod::Size{width, height, 0.0f};

        // Parse color
        auto *color_elem = find_element(j, "color");
        if (color_elem) {
            json_object_s *color_obj = get_object(color_elem->value);
            pigment::RGB machine_color = color.value_or(parse_color(color_obj));
            machine.color = machine_color;
        } else if (color.has_value()) {
            machine.color = color.value();
        }

        // Parse wheels
        auto *wheels_elem = find_element(j, "wheels");
        if (wheels_elem) {
            json_array_s *wheels = get_array(wheels_elem->value);
            if (wheels) {
                parse_wheels(machine, wheels);
            }
        }

        // Parse controls
        auto *controls_elem = find_element(j, "controls");
        if (controls_elem) {
            json_object_s *controls = get_object(controls_elem->value);
            if (controls) {
                parse_controls(machine, controls);
            }
        }

        // Parse optional sections
        auto *karosseries_elem = find_element(j, "karosseries");
        if (karosseries_elem) {
            json_array_s *karosseries = get_array(karosseries_elem->value);
            if (karosseries) {
                parse_karosseries(machine, karosseries, machine.color);
            }
        }

        auto *hitches_elem = find_element(j, "hitches");
        if (hitches_elem) {
            json_object_s *hitches = get_object(hitches_elem->value);
            if (hitches) {
                parse_hitches(machine, hitches);
            }
        }

        auto *tank_elem = find_element(j, "tank");
        if (tank_elem) {
            json_object_s *tank = get_object(tank_elem->value);
            if (tank) {
                parse_tank(machine, tank);
            }
        }

        auto *power_elem = find_element(j, "power");
        if (power_elem) {
            json_object_s *power = get_object(power_elem->value);
            if (power) {
                parse_power(machine, power);
            }
        }

        auto *capability_elem = find_element(j, "capability");
        if (capability_elem) {
            json_object_s *capability = get_object(capability_elem->value);
            if (capability) {
                parse_capability(machine, capability);
            }
        }

        auto *turn_elem = find_element(j, "turn");
        if (turn_elem) {
            json_object_s *turn = get_object(turn_elem->value);
            if (turn) {
                auto *radius_elem = find_element(turn, "radius");
                if (radius_elem) {
                    machine.turning_radius = static_cast<float>(get_number(radius_elem->value));
                }
            }
        }

        return machine;
    }

    std::vector<std::filesystem::path> Loader::find_machine_files(const std::filesystem::path &directory) {
        std::vector<std::filesystem::path> machine_files;

        if (!std::filesystem::exists(directory)) {
            return machine_files;
        }

        for (const auto &entry : std::filesystem::directory_iterator(directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                machine_files.push_back(entry.path());
            }
        }

        return machine_files;
    }

    bool Loader::validate_json(const std::filesystem::path &json_path) {
        try {
            std::ifstream file(json_path);
            if (!file.is_open()) {
                return false;
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string json_str = buffer.str();

            json_value_s *root = json_parse(json_str.c_str(), json_str.size());
            if (!root) {
                return false;
            }
            JsonPtr jv(root);

            json_object_s *j = get_object(jv.get());
            if (!j) {
                return false;
            }

            if (!find_element(j, "info") || !find_element(j, "dimensions") || !find_element(j, "color") ||
                !find_element(j, "wheels") || !find_element(j, "controls")) {
                return false;
            }

            auto *info_elem = find_element(j, "info");
            json_object_s *info = get_object(info_elem->value);
            if (!info) {
                return false;
            }

            if (!find_element(info, "type") || !find_element(info, "name") || !find_element(info, "rci") ||
                !find_element(info, "works_on")) {
                return false;
            }

            return true;
        } catch (const std::exception &e) {
            std::cerr << "JSON validation failed for " << json_path.string() << ": " << e.what() << std::endl;
            return false;
        }
    }

    pigment::RGB Loader::parse_color(json_object_s *color_json) {
        if (!color_json) {
            return pigment::RGB(0, 0, 0);
        }

        auto *r_elem = find_element(color_json, "r");
        auto *g_elem = find_element(color_json, "g");
        auto *b_elem = find_element(color_json, "b");

        int r = r_elem ? get_int(r_elem->value) : 0;
        int g = g_elem ? get_int(g_elem->value) : 0;
        int b = b_elem ? get_int(b_elem->value) : 0;

        return pigment::RGB(r, g, b);
    }

    datapod::Pose Loader::parse_pose(json_object_s *pos_json) {
        if (!pos_json) {
            return utils::make_pose(0.0, 0.0, 0.0, 0.0);
        }

        auto *x_elem = find_element(pos_json, "x");
        auto *y_elem = find_element(pos_json, "y");
        auto *yaw_elem = find_element(pos_json, "yaw");

        float x = x_elem ? static_cast<float>(get_number(x_elem->value)) : 0.0f;
        float y = y_elem ? static_cast<float>(get_number(y_elem->value)) : 0.0f;
        float yaw = yaw_elem ? static_cast<float>(get_number(yaw_elem->value)) : 0.0f;

        return utils::make_pose(x, y, 0.0, yaw);
    }

    datapod::Size Loader::parse_size(json_object_s *size_json) {
        if (!size_json) {
            return datapod::Size{0.0f, 0.0f, 0.0f};
        }

        auto *width_elem = find_element(size_json, "width");
        auto *height_elem = find_element(size_json, "height");
        auto *depth_elem = find_element(size_json, "depth");

        float width = width_elem ? static_cast<float>(get_number(width_elem->value)) : 0.0f;
        float height = height_elem ? static_cast<float>(get_number(height_elem->value)) : 0.0f;
        float depth = depth_elem ? static_cast<float>(get_number(depth_elem->value)) : 0.0f;

        return datapod::Size{width, height, depth};
    }

    void Loader::parse_wheels(types::Machine &machine, json_array_s *wheels_json) {
        if (!wheels_json) {
            return;
        }

        std::vector<bool> left_side;

        for (auto *wheel_elem = wheels_json->start; wheel_elem; wheel_elem = wheel_elem->next) {
            json_object_s *wheel = get_object(wheel_elem->value);
            if (!wheel) {
                continue;
            }

            types::Wheel w;

            auto *name_elem = find_element(wheel, "name");
            if (name_elem) {
                w.name = get_string(name_elem->value);
            }

            auto *position_elem = find_element(wheel, "position");
            if (position_elem) {
                json_object_s *position = get_object(position_elem->value);
                w.bound.pose = parse_pose(position);
            }

            auto *size_elem = find_element(wheel, "size");
            if (size_elem) {
                json_object_s *size = get_object(size_elem->value);
                w.bound.size = parse_size(size);
            }

            auto *color_elem = find_element(wheel, "color");
            if (color_elem) {
                json_object_s *color = get_object(color_elem->value);
                w.color = parse_color(color);
            } else {
                w.color = pigment::RGB(0, 0, 0);
            }

            machine.wheels.push_back(w);

            auto *side_elem = find_element(wheel, "side");
            std::string side = side_elem ? get_string(side_elem->value) : "";
            left_side.push_back(side == "left");
        }

        machine.controls.left_side = left_side;
    }

    void Loader::parse_controls(types::Machine &machine, json_object_s *controls_json) {
        if (!controls_json) {
            return;
        }

        auto *steering_elem = find_element(controls_json, "steering");
        auto *throttle_elem = find_element(controls_json, "throttle");

        if (steering_elem) {
            json_object_s *steering = get_object(steering_elem->value);
            if (steering) {
                auto *max_angles_elem = find_element(steering, "max_angles");
                if (max_angles_elem) {
                    json_array_s *max_angles = get_array(max_angles_elem->value);
                    if (max_angles) {
                        size_t i = 0;
                        for (auto *elem = max_angles->start; elem; elem = elem->next, ++i) {
                            float angle_rad = deg2rad(static_cast<float>(get_number(elem->value)));
                            machine.controls.steerings_max.push_back(angle_rad);
                            if (i < machine.wheels.size()) {
                                machine.wheels[i].steering_max = angle_rad;
                            }
                        }
                    }
                }

                auto *differential_elem = find_element(steering, "differential");
                if (differential_elem) {
                    json_array_s *differential = get_array(differential_elem->value);
                    if (differential) {
                        for (auto *elem = differential->start; elem; elem = elem->next) {
                            machine.controls.steerings_diff.push_back(
                                deg2rad(static_cast<float>(get_number(elem->value))));
                        }
                    }
                }
            }
        }

        if (throttle_elem) {
            json_object_s *throttle = get_object(throttle_elem->value);
            if (throttle) {
                auto *max_values_elem = find_element(throttle, "max_values");
                if (max_values_elem) {
                    json_array_s *max_values = get_array(max_values_elem->value);
                    if (max_values) {
                        size_t i = 0;
                        for (auto *elem = max_values->start; elem; elem = elem->next, ++i) {
                            float throttle_val = static_cast<float>(get_number(elem->value));
                            machine.controls.throttles_max.push_back(throttle_val);
                            if (i < machine.wheels.size()) {
                                machine.wheels[i].throttle_max = throttle_val;
                            }
                        }
                    }
                }

                auto *throttle_diff_elem = find_element(throttle, "differential");
                if (throttle_diff_elem) {
                    json_array_s *throttle_diff = get_array(throttle_diff_elem->value);
                    if (throttle_diff) {
                        for (auto *elem = throttle_diff->start; elem; elem = elem->next) {
                            machine.controls.throttles_diff.push_back(static_cast<float>(get_number(elem->value)));
                        }
                    }
                } else {
                    machine.controls.throttles_diff.resize(machine.controls.throttles_max.size(), 0.0f);
                }
            }
        }
    }

    void Loader::parse_karosseries(types::Machine &machine, json_array_s *karos_json, pigment::RGB default_color) {
        if (!karos_json) {
            return;
        }

        for (auto *karo_elem = karos_json->start; karo_elem; karo_elem = karo_elem->next) {
            json_object_s *karo = get_object(karo_elem->value);
            if (!karo) {
                continue;
            }

            types::Karosserie kaross;

            auto *name_elem = find_element(karo, "name");
            if (name_elem) {
                kaross.name = get_string(name_elem->value);
            }

            auto *position_elem = find_element(karo, "position");
            if (position_elem) {
                json_object_s *position = get_object(position_elem->value);
                kaross.bound.pose = parse_pose(position);
            }

            auto *size_elem = find_element(karo, "size");
            if (size_elem) {
                json_object_s *size = get_object(size_elem->value);
                kaross.bound.size = parse_size(size);
            }

            auto *color_elem = find_element(karo, "color");
            if (color_elem) {
                json_object_s *color = get_object(color_elem->value);
                kaross.color = parse_color(color);
            } else {
                kaross.color = default_color;
            }

            auto *has_physics_elem = find_element(karo, "has_physics");
            kaross.has_physics = has_physics_elem ? get_bool(has_physics_elem->value) : true;

            auto *sections_elem = find_element(karo, "sections");
            int sections_count = sections_elem ? get_int(sections_elem->value) : 0;
            for (int i = 0; i < sections_count; i++) {
                types::Section section;
                section.name = "section_" + std::to_string(i);
                section.bound.pose = utils::make_pose(0.0, 0.0, 0.0, 0.0);
                section.bound.size = kaross.bound.size;
                section.color = kaross.color;
                kaross.sections.push_back(section);
            }

            machine.karosseries.push_back(kaross);
        }
    }

    void Loader::parse_hitches(types::Machine &machine, json_object_s *hitches_json) {
        if (!hitches_json) {
            return;
        }

        for (auto *elem = hitches_json->start; elem; elem = elem->next) {
            std::string name(elem->name->string, elem->name->string_size);
            json_object_s *hitch = get_object(elem->value);
            if (!hitch) {
                continue;
            }

            types::Hitch hitch_info;
            hitch_info.name = name;

            auto *position_elem = find_element(hitch, "position");
            if (position_elem) {
                json_object_s *position = get_object(position_elem->value);
                hitch_info.bound.pose = parse_pose(position);
            }

            auto *size_elem = find_element(hitch, "size");
            if (size_elem) {
                json_object_s *size = get_object(size_elem->value);
                hitch_info.bound.size = parse_size(size);
            }

            hitch_info.color = pigment::RGB(0, 0, 0);

            auto *is_master_elem = find_element(hitch, "is_master");
            hitch_info.is_master = is_master_elem ? get_bool(is_master_elem->value) : true;

            machine.hitches[name] = hitch_info;
        }
    }

    void Loader::parse_tank(types::Machine &machine, json_object_s *tank_json) {
        if (!tank_json) {
            return;
        }

        types::Tank tank;

        auto *name_elem = find_element(tank_json, "name");
        if (name_elem) {
            tank.name = get_string(name_elem->value);
        }

        auto *type_elem = find_element(tank_json, "type");
        if (type_elem) {
            std::string type_str = get_string(type_elem->value);
            tank.type = (type_str == "WASTE") ? types::ContainerType::WASTE : types::ContainerType::HARVEST;
        }

        auto *capacity_elem = find_element(tank_json, "capacity");
        if (capacity_elem) {
            tank.capacity = static_cast<float>(get_number(capacity_elem->value));
        }

        auto *position_elem = find_element(tank_json, "position");
        if (position_elem) {
            json_object_s *position = get_object(position_elem->value);
            tank.bound.pose = parse_pose(position);
        }

        auto *size_elem = find_element(tank_json, "size");
        if (size_elem) {
            json_object_s *size = get_object(size_elem->value);
            tank.bound.size = parse_size(size);
        }

        machine.tank = tank;
    }

    void Loader::parse_power(types::Machine &machine, json_object_s *power_json) {
        if (!power_json) {
            return;
        }

        types::Power power;

        auto *name_elem = find_element(power_json, "name");
        if (name_elem) {
            power.name = get_string(name_elem->value);
        }

        auto *type_elem = find_element(power_json, "type");
        if (type_elem) {
            std::string type_str = get_string(type_elem->value);
            power.type = (type_str == "BATTERY") ? types::PowerType::BATTERY : types::PowerType::FUEL;
        }

        auto *capacity_elem = find_element(power_json, "capacity");
        if (capacity_elem) {
            power.capacity = static_cast<float>(get_number(capacity_elem->value));
        }

        auto *consumption_rate_elem = find_element(power_json, "consumption_rate");
        if (consumption_rate_elem) {
            power.consumption_rate = static_cast<float>(get_number(consumption_rate_elem->value));
        }

        auto *charge_rate_elem = find_element(power_json, "charge_rate");
        power.charge_rate = charge_rate_elem ? static_cast<float>(get_number(charge_rate_elem->value)) : 0.0f;

        machine.power_source = power;
    }

    void Loader::parse_capability(types::Machine &machine, json_object_s *capability_json) {
        if (!capability_json) {
            return;
        }

        auto *work_on_elem = find_element(capability_json, "work_on");
        if (work_on_elem) {
            json_array_s *work_on = get_array(work_on_elem->value);
            if (work_on) {
                for (auto *elem = work_on->start; elem; elem = elem->next) {
                    machine.capability.work_on.push_back(get_string(elem->value));
                }
            }
        }

        auto *connect_to_elem = find_element(capability_json, "connect_to");
        if (connect_to_elem) {
            json_array_s *connect_to = get_array(connect_to_elem->value);
            if (connect_to) {
                for (auto *elem = connect_to->start; elem; elem = elem->next) {
                    machine.capability.connect_to.push_back(get_string(elem->value));
                }
            }
        }

        auto *unload_to_elem = find_element(capability_json, "unload_to");
        if (unload_to_elem) {
            json_array_s *unload_to = get_array(unload_to_elem->value);
            if (unload_to) {
                for (auto *elem = unload_to->start; elem; elem = elem->next) {
                    machine.capability.unload_to.push_back(get_string(elem->value));
                }
            }
        }
    }

} // namespace agent
