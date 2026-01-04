#pragma once

#include "flatsim/agent/loader/json.hpp"
#include "flatsim/types.hpp"
#include <filesystem>
#include <optional>
#include <string>

namespace agent {

    class Loader {
      public:
        static types::Machine load_from_json(const std::filesystem::path &json_path, datapod::Pose spawn_pose,
                                             std::optional<pigment::RGB> color = std::nullopt);

        static std::vector<std::filesystem::path> find_machine_files(const std::filesystem::path &directory);
        static bool validate_json(const std::filesystem::path &json_path);

      private:
        static pigment::RGB parse_color(json_object_s *color_json);
        static datapod::Pose parse_pose(json_object_s *pos_json);
        static datapod::Size parse_size(json_object_s *size_json);
        static void parse_wheels(types::Machine &machine, json_array_s *wheels_json);
        static void parse_controls(types::Machine &machine, json_object_s *controls_json);
        static void parse_karosseries(types::Machine &machine, json_array_s *karos_json, pigment::RGB default_color);
        static void parse_hitches(types::Machine &machine, json_object_s *hitches_json);
        static void parse_tank(types::Machine &machine, json_object_s *tank_json);
        static void parse_power(types::Machine &machine, json_object_s *power_json);
        static void parse_capability(types::Machine &machine, json_object_s *capability_json);

        static std::string generate_uuid();

        // Helper functions for JSON parsing
        static json_object_element_s *find_element(json_object_s *obj, const char *key);
        static std::string get_string(json_value_s *val);
        static double get_number(json_value_s *val);
        static int get_int(json_value_s *val);
        static bool get_bool(json_value_s *val);
        static json_object_s *get_object(json_value_s *val);
        static json_array_s *get_array(json_value_s *val);
    };

} // namespace agent
