#pragma once

#include "flatsim/types.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <optional>

namespace fs {

    class Loader {
      public:
        static RobotInfo load_from_json(const std::filesystem::path &json_path, concord::Pose spawn_pose,
                                        const std::string &name = "", std::optional<pigment::RGB> color = std::nullopt);

        static std::vector<std::filesystem::path> find_machine_files(const std::filesystem::path &directory);
        static bool validate_json(const std::filesystem::path &json_path);

      private:
        static pigment::RGB parse_color(const nlohmann::json &color_json);
        static concord::Pose parse_pose(const nlohmann::json &pos_json);
        static concord::Size parse_size(const nlohmann::json &size_json);
        static void parse_wheels(RobotInfo &info, const nlohmann::json &wheels_json);
        static void parse_controls(RobotInfo &info, const nlohmann::json &controls_json);
        static void parse_karosseries(RobotInfo &info, const nlohmann::json &karos_json, pigment::RGB default_color);
        static void parse_hitches(RobotInfo &info, const nlohmann::json &hitches_json);
        static void parse_tank(RobotInfo &info, const nlohmann::json &tank_json);
        static void parse_power(RobotInfo &info, const nlohmann::json &power_json);
        static void parse_capability(RobotInfo &info, const nlohmann::json &capability_json);
    };

} // namespace fs
