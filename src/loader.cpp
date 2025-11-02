#include "flatsim/loader.hpp"
#include "flatsim/utils.hpp"
#include <boost/json.hpp>
#include <fstream>
#include <iomanip>
#include <random>
#include <spdlog/spdlog.h>
#include <sstream>

namespace fs {

    // Helper functions for Boost.JSON
    template <typename T> static T get_value(const boost::json::value &v) { return boost::json::value_to<T>(v); }

    template <typename T> static T get_value_or(const boost::json::object &obj, const char *key, T default_val) {
        if (obj.contains(key)) {
            return boost::json::value_to<T>(obj.at(key));
        }
        return default_val;
    }

    // Generate a proper UUID (UUID v4 format)
    static std::string generate_uuid() {
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
        ss << "-4"; // UUID version 4
        for (int i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        ss << dis2(gen); // UUID variant
        for (int i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (int i = 0; i < 12; i++) {
            ss << dis(gen);
        }
        return ss.str();
    }

    RobotInfo Loader::load_from_json(const std::filesystem::path &json_path, concord::Pose spawn_pose,
                                     const std::string &name, std::optional<pigment::RGB> color) {
        spdlog::info("Loading machine from: {}", json_path.string());

        // Read JSON file
        std::ifstream file(json_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open machine file: " + json_path.string());
        }

        std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();

        boost::json::value jv = boost::json::parse(json_str);
        boost::json::object const &j = jv.as_object();

        RobotInfo robot_info;

        // Parse basic info
        boost::json::object const &info = j.at("info").as_object();
        robot_info.type = get_value<std::string>(info.at("type"));
        std::string default_name = get_value<std::string>(info.at("name"));
        robot_info.name = name.empty() ? default_name : name;

        // Generate proper UUID if not provided or empty
        std::string uuid_str = get_value_or(info, "uuid", std::string(""));
        robot_info.uuid = (uuid_str.empty() || uuid_str == "") ? generate_uuid() : uuid_str;
        robot_info.RCI = get_value<uint>(info.at("rci"));

        // Parse works_on array
        boost::json::array const &works_on = info.at("works_on").as_array();
        for (const auto &work : works_on) {
            robot_info.works_on.push_back(get_value<std::string>(work));
        }

        // Parse role
        std::string role_str = get_value_or(info, "role", std::string("MASTER"));
        if (role_str == "SLAVE") {
            robot_info.role = RobotRole::SLAVE;
        } else if (role_str == "FOLLOWER") {
            robot_info.role = RobotRole::FOLLOWER;
        } else {
            robot_info.role = RobotRole::MASTER;
        }

        // Parse dimensions
        boost::json::object const &dims = j.at("dimensions").as_object();
        float width = get_value<float>(dims.at("width"));
        float height = get_value<float>(dims.at("height"));
        robot_info.bound = concord::Bound(spawn_pose, concord::Size(width, height, 0.0f));

        // Parse color (use override if provided)
        pigment::RGB robot_color = color.value_or(parse_color(j.at("color").as_object()));
        robot_info.color = robot_color;

        // Parse wheels
        parse_wheels(robot_info, j.at("wheels").as_array());

        // Parse controls
        parse_controls(robot_info, j.at("controls").as_object());

        // Parse karosseries
        if (j.contains("karosseries")) {
            parse_karosseries(robot_info, j.at("karosseries").as_array(), robot_color);
        }

        // Parse hitches
        if (j.contains("hitches")) {
            parse_hitches(robot_info, j.at("hitches").as_object());
        }

        // Parse tank
        if (j.contains("tank")) {
            parse_tank(robot_info, j.at("tank").as_object());
        }

        // Parse power
        if (j.contains("power")) {
            parse_power(robot_info, j.at("power").as_object());
        }

        // Parse capability
        if (j.contains("capability")) {
            parse_capability(robot_info, j.at("capability").as_object());
        }

        // Parse turning radius
        if (j.contains("turn")) {
            boost::json::object const &turn = j.at("turn").as_object();
            if (turn.contains("radius")) {
                robot_info.turning_radius = get_value<float>(turn.at("radius"));
            }
        }

        return robot_info;
    }

    std::vector<std::filesystem::path> Loader::find_machine_files(const std::filesystem::path &directory) {
        std::vector<std::filesystem::path> machine_files;

        if (!std::filesystem::exists(directory)) {
            spdlog::warn("Machine directory does not exist: {}", directory.string());
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

            // Check required fields
            if (!j.contains("info") || !j.contains("dimensions") || !j.contains("color") || !j.contains("wheels") ||
                !j.contains("controls")) {
                return false;
            }

            // Check info fields
            boost::json::object const &info = j.at("info").as_object();
            if (!info.contains("type") || !info.contains("name") || !info.contains("rci") ||
                !info.contains("works_on")) {
                return false;
            }

            return true;
        } catch (const std::exception &e) {
            spdlog::error("JSON validation failed for {}: {}", json_path.string(), e.what());
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

    void Loader::parse_wheels(RobotInfo &info, const boost::json::array &wheels_json) {
        std::vector<concord::Bound> wheels;
        std::vector<bool> left_side;

        for (const auto &wheel_val : wheels_json) {
            boost::json::object const &wheel = wheel_val.as_object();
            concord::Pose wheel_pose = parse_pose(wheel.at("position").as_object());
            concord::Size wheel_size = parse_size(wheel.at("size").as_object());
            wheels.push_back(concord::Bound(wheel_pose, wheel_size));

            std::string side = get_value<std::string>(wheel.at("side"));
            left_side.push_back(side == "left");
        }

        info.wheels = wheels;
        info.controls.left_side = left_side;
    }

    void Loader::parse_controls(RobotInfo &info, const boost::json::object &controls_json) {
        boost::json::object const &steering = controls_json.at("steering").as_object();
        boost::json::object const &throttle = controls_json.at("throttle").as_object();

        boost::json::array const &max_angles = steering.at("max_angles").as_array();
        for (const auto &angle : max_angles) {
            info.controls.steerings_max.push_back(utils::deg2rad(get_value<float>(angle)));
        }

        boost::json::array const &differential = steering.at("differential").as_array();
        for (const auto &diff : differential) {
            info.controls.steerings_diff.push_back(utils::deg2rad(get_value<float>(diff)));
        }

        boost::json::array const &max_values = throttle.at("max_values").as_array();
        for (const auto &value : max_values) {
            info.controls.throttles_max.push_back(get_value<float>(value));
        }

        // Parse throttle differential if present
        if (throttle.contains("differential")) {
            boost::json::array const &throttle_diff = throttle.at("differential").as_array();
            for (const auto &diff : throttle_diff) {
                info.controls.throttles_diff.push_back(get_value<float>(diff));
            }
        } else {
            // Default to no throttle differential
            info.controls.throttles_diff.resize(info.controls.throttles_max.size(), 0.0f);
        }
    }

    void Loader::parse_karosseries(RobotInfo &info, const boost::json::array &karos_json, pigment::RGB default_color) {
        for (const auto &karo_val : karos_json) {
            boost::json::object const &karo = karo_val.as_object();
            KarosserieInfo kaross;
            kaross.name = get_value<std::string>(karo.at("name"));
            kaross.bound =
                concord::Bound(parse_pose(karo.at("position").as_object()), parse_size(karo.at("size").as_object()));
            kaross.color = karo.contains("color") ? parse_color(karo.at("color").as_object()) : default_color;
            kaross.sections = get_value_or(karo, "sections", 0);
            kaross.has_physics = get_value_or(karo, "has_physics", true);

            info.karos.push_back(kaross);
        }
    }

    void Loader::parse_hitches(RobotInfo &info, const boost::json::object &hitches_json) {
        for (auto const &item : hitches_json) {
            std::string name = std::string(item.key());
            boost::json::object const &hitch = item.value().as_object();

            concord::Pose hitch_pose = parse_pose(hitch.at("position").as_object());
            concord::Size hitch_size = parse_size(hitch.at("size").as_object());
            bool is_master = get_value_or(hitch, "is_master", true);

            HitchInfo hitch_info;
            hitch_info.bound = concord::Bound(hitch_pose, hitch_size);
            hitch_info.is_master = is_master;

            info.hitches[name] = hitch_info;
        }
    }

    void Loader::parse_tank(RobotInfo &info, const boost::json::object &tank_json) {
        TankInfo tank;
        tank.name = get_value<std::string>(tank_json.at("name"));
        tank.capacity = get_value<float>(tank_json.at("capacity"));
        tank.bound = concord::Bound(parse_pose(tank_json.at("position").as_object()),
                                    parse_size(tank_json.at("size").as_object()));

        info.tank = tank;
    }

    void Loader::parse_power(RobotInfo &info, const boost::json::object &power_json) {
        PowerInfo power;
        power.name = get_value<std::string>(power_json.at("name"));

        std::string type_str = get_value<std::string>(power_json.at("type"));
        power.type = (type_str == "BATTERY") ? PowerType::BATTERY : PowerType::FUEL;

        power.capacity = get_value<float>(power_json.at("capacity"));
        power.consumption_rate = get_value<float>(power_json.at("consumption_rate"));
        power.charge_rate = get_value_or(power_json, "charge_rate", 0.0f);

        info.power_source = power;
    }

    void Loader::parse_capability(RobotInfo &info, const boost::json::object &capability_json) {
        if (capability_json.contains("work_on")) {
            boost::json::array const &work_on = capability_json.at("work_on").as_array();
            for (const auto &work : work_on) {
                info.capability.work_on.push_back(get_value<std::string>(work));
            }
        }

        if (capability_json.contains("connect_to")) {
            boost::json::array const &connect_to = capability_json.at("connect_to").as_array();
            for (const auto &connect : connect_to) {
                info.capability.connect_to.push_back(get_value<std::string>(connect));
            }
        }

        if (capability_json.contains("unload_to")) {
            boost::json::array const &unload_to = capability_json.at("unload_to").as_array();
            for (const auto &unload : unload_to) {
                info.capability.unload_to.push_back(get_value<std::string>(unload));
            }
        }
    }

} // namespace fs
