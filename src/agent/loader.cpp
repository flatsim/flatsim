#include "flatsim/agent/loader.hpp"
#include <boost/json.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

namespace agent {

    template <typename T> static T get_value(const boost::json::value &v) { return boost::json::value_to<T>(v); }

    template <typename T> static T get_value_or(const boost::json::object &obj, const char *key, T default_val) {
        if (obj.contains(key)) {
            return boost::json::value_to<T>(obj.at(key));
        }
        return default_val;
    }

    static float deg2rad(float deg) { return deg * (M_PI / 180.0f); }

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

    types::Machine Loader::load_from_json(const std::filesystem::path &json_path, concord::Pose spawn_pose,
                                          std::optional<pigment::RGB> color) {

        std::ifstream file(json_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open machine file: " + json_path.string());
        }

        std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();

        boost::json::value jv = boost::json::parse(json_str);
        boost::json::object const &j = jv.as_object();

        types::Machine machine;

        boost::json::object const &info = j.at("info").as_object();
        machine.type = get_value<std::string>(info.at("type"));
        machine.name = get_value<std::string>(info.at("name"));

        std::string uuid_str = get_value_or(info, "uuid", std::string(""));
        machine.uuid = (uuid_str.empty() || uuid_str == "") ? generate_uuid() : uuid_str;
        machine.rci = get_value<uint32_t>(info.at("rci"));

        boost::json::array const &works_on = info.at("works_on").as_array();
        for (const auto &work : works_on) {
            machine.works_on.push_back(get_value<std::string>(work));
        }

        std::string role_str = get_value_or(info, "role", std::string("MASTER"));
        if (role_str == "SLAVE") {
            machine.role = types::MachineRole::SLAVE;
        } else if (role_str == "FOLLOWER") {
            machine.role = types::MachineRole::FOLLOWER;
        } else {
            machine.role = types::MachineRole::MASTER;
        }

        boost::json::object const &dims = j.at("dimensions").as_object();
        float width = get_value<float>(dims.at("width"));
        float height = get_value<float>(dims.at("height"));
        machine.bound.pose = spawn_pose;
        machine.bound.size = concord::Size(width, height, 0.0f);

        pigment::RGB machine_color = color.value_or(parse_color(j.at("color").as_object()));
        machine.color = machine_color;

        parse_wheels(machine, j.at("wheels").as_array());

        parse_controls(machine, j.at("controls").as_object());

        if (j.contains("karosseries")) {
            parse_karosseries(machine, j.at("karosseries").as_array(), machine_color);
        }

        if (j.contains("hitches")) {
            parse_hitches(machine, j.at("hitches").as_object());
        }

        if (j.contains("tank")) {
            parse_tank(machine, j.at("tank").as_object());
        }

        if (j.contains("power")) {
            parse_power(machine, j.at("power").as_object());
        }

        if (j.contains("capability")) {
            parse_capability(machine, j.at("capability").as_object());
        }

        if (j.contains("turn")) {
            boost::json::object const &turn = j.at("turn").as_object();
            if (turn.contains("radius")) {
                machine.turning_radius = get_value<float>(turn.at("radius"));
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

            std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            boost::json::value jv = boost::json::parse(json_str);
            boost::json::object const &j = jv.as_object();

            if (!j.contains("info") || !j.contains("dimensions") || !j.contains("color") || !j.contains("wheels") ||
                !j.contains("controls")) {
                return false;
            }

            boost::json::object const &info = j.at("info").as_object();
            if (!info.contains("type") || !info.contains("name") || !info.contains("rci") ||
                !info.contains("works_on")) {
                return false;
            }

            return true;
        } catch (const std::exception &e) {
            std::cerr << "JSON validation failed for " << json_path.string() << ": " << e.what() << std::endl;
            return false;
        }
    }

    pigment::RGB Loader::parse_color(const boost::json::object &color_json) {
        return pigment::RGB(get_value<int>(color_json.at("r")), get_value<int>(color_json.at("g")),
                            get_value<int>(color_json.at("b")));
    }

    concord::Pose Loader::parse_pose(const boost::json::object &pos_json) {
        float x = get_value<float>(pos_json.at("x"));
        float y = get_value<float>(pos_json.at("y"));
        float yaw = get_value_or(pos_json, "yaw", 0.0f);
        return concord::Pose(x, y, yaw);
    }

    concord::Size Loader::parse_size(const boost::json::object &size_json) {
        float width = get_value<float>(size_json.at("width"));
        float height = get_value<float>(size_json.at("height"));
        float depth = get_value_or(size_json, "depth", 0.0f);
        return concord::Size(width, height, depth);
    }

    void Loader::parse_wheels(types::Machine &machine, const boost::json::array &wheels_json) {
        std::vector<bool> left_side;

        for (const auto &wheel_val : wheels_json) {
            boost::json::object const &wheel = wheel_val.as_object();

            types::Wheel w;
            w.name = get_value<std::string>(wheel.at("name"));
            w.bound.pose = parse_pose(wheel.at("position").as_object());
            w.bound.size = parse_size(wheel.at("size").as_object());
            w.color = wheel.contains("color") ? parse_color(wheel.at("color").as_object()) : pigment::RGB(0, 0, 0);

            machine.wheels.push_back(w);

            std::string side = get_value<std::string>(wheel.at("side"));
            left_side.push_back(side == "left");
        }

        machine.controls.left_side = left_side;
    }

    void Loader::parse_controls(types::Machine &machine, const boost::json::object &controls_json) {
        boost::json::object const &steering = controls_json.at("steering").as_object();
        boost::json::object const &throttle = controls_json.at("throttle").as_object();

        boost::json::array const &max_angles = steering.at("max_angles").as_array();
        for (const auto &angle : max_angles) {
            machine.controls.steerings_max.push_back(deg2rad(get_value<float>(angle)));
        }

        boost::json::array const &differential = steering.at("differential").as_array();
        for (const auto &diff : differential) {
            machine.controls.steerings_diff.push_back(deg2rad(get_value<float>(diff)));
        }

        boost::json::array const &max_values = throttle.at("max_values").as_array();
        for (const auto &value : max_values) {
            machine.controls.throttles_max.push_back(get_value<float>(value));
        }

        if (throttle.contains("differential")) {
            boost::json::array const &throttle_diff = throttle.at("differential").as_array();
            for (const auto &diff : throttle_diff) {
                machine.controls.throttles_diff.push_back(get_value<float>(diff));
            }
        } else {
            machine.controls.throttles_diff.resize(machine.controls.throttles_max.size(), 0.0f);
        }
    }

    void Loader::parse_karosseries(types::Machine &machine, const boost::json::array &karos_json,
                                   pigment::RGB default_color) {
        for (const auto &karo_val : karos_json) {
            boost::json::object const &karo = karo_val.as_object();
            types::Karosserie kaross;
            kaross.name = get_value<std::string>(karo.at("name"));
            kaross.bound.pose = parse_pose(karo.at("position").as_object());
            kaross.bound.size = parse_size(karo.at("size").as_object());
            kaross.color = karo.contains("color") ? parse_color(karo.at("color").as_object()) : default_color;
            kaross.has_physics = get_value_or(karo, "has_physics", true);

            int sections_count = get_value_or(karo, "sections", 0);
            for (int i = 0; i < sections_count; i++) {
                types::Section section;
                section.name = "section_" + std::to_string(i);
                section.bound.pose = concord::Pose(0.0, 0.0, 0.0);
                section.bound.size = kaross.bound.size;
                section.color = kaross.color;
                kaross.sections.push_back(section);
            }

            machine.karosseries.push_back(kaross);
        }
    }

    void Loader::parse_hitches(types::Machine &machine, const boost::json::object &hitches_json) {
        for (auto const &item : hitches_json) {
            std::string name = std::string(item.key());
            boost::json::object const &hitch = item.value().as_object();

            types::Hitch hitch_info;
            hitch_info.name = name;
            hitch_info.bound.pose = parse_pose(hitch.at("position").as_object());
            hitch_info.bound.size = parse_size(hitch.at("size").as_object());
            hitch_info.color = pigment::RGB(0, 0, 0);
            hitch_info.is_master = get_value_or(hitch, "is_master", true);

            machine.hitches[name] = hitch_info;
        }
    }

    void Loader::parse_tank(types::Machine &machine, const boost::json::object &tank_json) {
        types::Tank tank;
        tank.name = get_value<std::string>(tank_json.at("name"));
        tank.capacity = get_value<float>(tank_json.at("capacity"));
        tank.bound.pose = parse_pose(tank_json.at("position").as_object());
        tank.bound.size = parse_size(tank_json.at("size").as_object());

        machine.tank = tank;
    }

    void Loader::parse_power(types::Machine &machine, const boost::json::object &power_json) {
        types::Power power;
        power.name = get_value<std::string>(power_json.at("name"));

        std::string type_str = get_value<std::string>(power_json.at("type"));
        power.type = (type_str == "BATTERY") ? types::PowerType::BATTERY : types::PowerType::FUEL;

        power.capacity = get_value<float>(power_json.at("capacity"));
        power.consumption_rate = get_value<float>(power_json.at("consumption_rate"));
        power.charge_rate = get_value_or(power_json, "charge_rate", 0.0f);

        machine.power_source = power;
    }

    void Loader::parse_capability(types::Machine &machine, const boost::json::object &capability_json) {
        if (capability_json.contains("work_on")) {
            boost::json::array const &work_on = capability_json.at("work_on").as_array();
            for (const auto &work : work_on) {
                machine.capability.work_on.push_back(get_value<std::string>(work));
            }
        }

        if (capability_json.contains("connect_to")) {
            boost::json::array const &connect_to = capability_json.at("connect_to").as_array();
            for (const auto &connect : connect_to) {
                machine.capability.connect_to.push_back(get_value<std::string>(connect));
            }
        }

        if (capability_json.contains("unload_to")) {
            boost::json::array const &unload_to = capability_json.at("unload_to").as_array();
            for (const auto &unload : unload_to) {
                machine.capability.unload_to.push_back(get_value<std::string>(unload));
            }
        }
    }

} // namespace agent
