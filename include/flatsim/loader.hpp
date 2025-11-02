#pragma once

#include "flatsim/types.hpp"
#include <boost/json.hpp>
#include <filesystem>
#include <optional>
#include <string>

namespace fs {

    class Loader {
      public:
        static RobotInfo load_from_json(const std::filesystem::path &json_path, concord::Pose spawn_pose,
                                        const std::string &name = "", std::optional<pigment::RGB> color = std::nullopt);

        static std::vector<std::filesystem::path> find_machine_files(const std::filesystem::path &directory);
        static bool validate_json(const std::filesystem::path &json_path);

      private:
        static pigment::RGB parse_color(const boost::json::object &color_json);
        static concord::Pose parse_pose(const boost::json::object &pos_json);
        static concord::Size parse_size(const boost::json::object &size_json);
        static void parse_wheels(RobotInfo &info, const boost::json::array &wheels_json);
        static void parse_controls(RobotInfo &info, const boost::json::object &controls_json);
        static void parse_karosseries(RobotInfo &info, const boost::json::array &karos_json,
                                      pigment::RGB default_color);
        static void parse_hitches(RobotInfo &info, const boost::json::object &hitches_json);
        static void parse_tank(RobotInfo &info, const boost::json::object &tank_json);
        static void parse_power(RobotInfo &info, const boost::json::object &power_json);
        static void parse_capability(RobotInfo &info, const boost::json::object &capability_json);
    };

} // namespace fs
