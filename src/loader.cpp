#include "flatsim/loader.hpp"
#include "flatsim/utils.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <spdlog/spdlog.h>

namespace fs {

using json = nlohmann::json;

RobotInfo Loader::load_from_json(const std::filesystem::path& json_path,
                                       concord::Pose spawn_pose,
                                       const std::string& name,
                                       std::optional<pigment::RGB> color) {
    spdlog::info("Loading machine from: {}", json_path.string());
    
    // Read JSON file
    std::ifstream file(json_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open machine file: " + json_path.string());
    }
    
    json j;
    file >> j;
    file.close();
    
    RobotInfo robot_info;
    
    // Parse basic info
    auto& info = j["info"];
    robot_info.type = info["type"];
    std::string default_name = info["name"].get<std::string>();
    robot_info.name = name.empty() ? default_name : name;
    
    // Use original name from JSON as default UUID if UUID is empty
    std::string uuid_str = info.value("uuid", "");
    robot_info.uuid = uuid_str.empty() ? default_name + "_" + std::to_string(std::rand()) : uuid_str;
    robot_info.RCI = info["rci"];
    
    // Parse works_on array
    for (const auto& work : info["works_on"]) {
        robot_info.works_on.push_back(work);
    }
    
    // Parse role
    std::string role_str = info.value("role", "MASTER");
    if (role_str == "SLAVE") {
        robot_info.role = RobotRole::SLAVE;
    } else if (role_str == "FOLLOWER") {
        robot_info.role = RobotRole::FOLLOWER;
    } else {
        robot_info.role = RobotRole::MASTER;
    }
    
    // Parse dimensions
    auto& dims = j["dimensions"];
    float width = dims["width"];
    float height = dims["height"];
    robot_info.bound = concord::Bound(spawn_pose, concord::Size(width, height, 0.0f));
    
    // Parse color (use override if provided)
    pigment::RGB robot_color = color.value_or(parse_color(j["color"]));
    robot_info.color = robot_color;
    
    // Parse wheels
    parse_wheels(robot_info, j["wheels"]);
    
    // Parse controls
    parse_controls(robot_info, j["controls"]);
    
    // Parse karosseries
    if (j.contains("karosseries")) {
        parse_karosseries(robot_info, j["karosseries"], robot_color);
    }
    
    // Parse hitches
    if (j.contains("hitches")) {
        parse_hitches(robot_info, j["hitches"]);
    }
    
    // Parse tank
    if (j.contains("tank")) {
        parse_tank(robot_info, j["tank"]);
    }
    
    // Parse power
    if (j.contains("power")) {
        parse_power(robot_info, j["power"]);
    }
    
    return robot_info;
}

std::vector<std::filesystem::path> Loader::find_machine_files(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> machine_files;
    
    if (!std::filesystem::exists(directory)) {
        spdlog::warn("Machine directory does not exist: {}", directory.string());
        return machine_files;
    }
    
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            machine_files.push_back(entry.path());
        }
    }
    
    return machine_files;
}

bool Loader::validate_json(const std::filesystem::path& json_path) {
    try {
        std::ifstream file(json_path);
        if (!file.is_open()) {
            return false;
        }
        
        json j;
        file >> j;
        
        // Check required fields
        if (!j.contains("info") || !j.contains("dimensions") || 
            !j.contains("color") || !j.contains("wheels") || 
            !j.contains("controls")) {
            return false;
        }
        
        // Check info fields
        auto& info = j["info"];
        if (!info.contains("type") || !info.contains("name") || 
            !info.contains("rci") || !info.contains("works_on")) {
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        spdlog::error("JSON validation failed for {}: {}", json_path.string(), e.what());
        return false;
    }
}

pigment::RGB Loader::parse_color(const json& color_json) {
    return pigment::RGB(
        color_json["r"].get<int>(),
        color_json["g"].get<int>(),
        color_json["b"].get<int>()
    );
}

concord::Pose Loader::parse_pose(const json& pos_json) {
    float x = pos_json["x"];
    float y = pos_json["y"];
    float yaw = pos_json.value("yaw", 0.0f);
    return concord::Pose(x, y, yaw);
}

concord::Size Loader::parse_size(const json& size_json) {
    float width = size_json["width"];
    float height = size_json["height"];
    float depth = size_json.value("depth", 0.0f);
    return concord::Size(width, height, depth);
}

void Loader::parse_wheels(RobotInfo& info, const json& wheels_json) {
    std::vector<concord::Bound> wheels;
    std::vector<bool> left_side;
    
    for (const auto& wheel : wheels_json) {
        concord::Pose wheel_pose = parse_pose(wheel["position"]);
        concord::Size wheel_size = parse_size(wheel["size"]);
        wheels.push_back(concord::Bound(wheel_pose, wheel_size));
        
        std::string side = wheel["side"];
        left_side.push_back(side == "left");
    }
    
    info.wheels = wheels;
    info.controls.left_side = left_side;
}

void Loader::parse_controls(RobotInfo& info, const json& controls_json) {
    auto& steering = controls_json["steering"];
    auto& throttle = controls_json["throttle"];
    
    for (const auto& angle : steering["max_angles"]) {
        info.controls.steerings_max.push_back(utils::deg2rad(angle.get<float>()));
    }
    
    for (const auto& diff : steering["differential"]) {
        info.controls.steerings_diff.push_back(utils::deg2rad(diff.get<float>()));
    }
    
    for (const auto& value : throttle["max_values"]) {
        info.controls.throttles_max.push_back(value.get<float>());
    }
}

void Loader::parse_karosseries(RobotInfo& info, const json& karos_json, pigment::RGB default_color) {
    for (const auto& karo : karos_json) {
        KarosserieInfo kaross;
        kaross.name = karo["name"];
        kaross.bound = concord::Bound(parse_pose(karo["position"]), parse_size(karo["size"]));
        kaross.color = karo.contains("color") ? parse_color(karo["color"]) : default_color;
        kaross.sections = karo.value("sections", 0);
        kaross.has_physics = karo.value("has_physics", true);
        
        info.karos.push_back(kaross);
    }
}

void Loader::parse_hitches(RobotInfo& info, const json& hitches_json) {
    for (auto& [name, hitch] : hitches_json.items()) {
        concord::Pose hitch_pose = parse_pose(hitch["position"]);
        concord::Size hitch_size = parse_size(hitch["size"]);
        info.hitches[name] = concord::Bound(hitch_pose, hitch_size);
    }
}

void Loader::parse_tank(RobotInfo& info, const json& tank_json) {
    TankInfo tank;
    tank.name = tank_json["name"];
    tank.capacity = tank_json["capacity"];
    tank.bound = concord::Bound(parse_pose(tank_json["position"]), parse_size(tank_json["size"]));
    
    info.tank = tank;
}

void Loader::parse_power(RobotInfo& info, const json& power_json) {
    PowerInfo power;
    power.name = power_json["name"];
    
    std::string type_str = power_json["type"];
    power.type = (type_str == "BATTERY") ? PowerType::BATTERY : PowerType::FUEL;
    
    power.capacity = power_json["capacity"];
    power.consumption_rate = power_json["consumption_rate"];
    power.charge_rate = power_json.value("charge_rate", 0.0f);
    
    info.power_source = power;
}

} // namespace fs