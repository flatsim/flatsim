#pragma once

#include "concord/concord.hpp"
#include "entropy/entropy.hpp"
#include "pigment/pigment.hpp"
#include "rerun.hpp"
#include <unordered_map>
#include <optional>
namespace fs {

    // ROBOT
    struct RobotControll {
        std::vector<float> steerings_max;
        std::vector<float> throttles_max;
        std::vector<float> steerings_diff;
        std::vector<float> throttles_diff;
        std::vector<bool> left_side;
    };

    struct KarosserieInfo {
        std::string name;
        concord::Bound bound;
        pigment::RGB color;
        int sections = 0;
        bool has_physics = true;
    };

    struct HitchInfo {
        concord::Bound bound;   // Position and size relative to robot center
        bool is_master = true;  // true = master (can pull), false = slave (can be pulled)
    };

    struct TankInfo {
        std::string name;
        float capacity;
        concord::Bound bound;  // Position and size relative to robot center
    };

    enum class PowerType {
        FUEL,
        BATTERY
    };

    struct PowerInfo {
        std::string name;
        PowerType type;
        float capacity;
        float consumption_rate;
        float charge_rate = 0.0f;  // only for batteries
    };

    enum class RobotRole { MASTER, FOLLOWER, SLAVE };

    struct Capability {
        std::vector<std::string> work_on;      // What this machine can work on
        std::vector<std::string> connect_to;   // What this machine can connect to
        std::vector<std::string> unload_to;    // What this machine can unload to
    };

    struct RobotInfo {
        uint RCI;
        uint group;
        bool slave = false;
        std::string name = "unnamed";
        std::string uuid = "none";
        std::string type = "none";
        std::vector<std::string> works_on;
        Capability capability;
        pigment::RGB color;
        concord::Bound bound;
        concord::Polygon outline;
        std::vector<concord::Bound> wheels;
        RobotControll controls;
        std::unordered_map<std::string, HitchInfo> hitches;
        std::vector<KarosserieInfo> karos;
        std::optional<TankInfo> tank;  // Optional tank (harvesters, biners)
        std::optional<PowerInfo> power_source;  // Optional power (not all machines need power)
        RobotRole role = RobotRole::MASTER;  // Default to MASTER
    };
    
    enum class OP { IDLE, CHARGING, STOP, PAUSE, EMERGENCY, TRANSPORT, WORK };
    // WORLD
    struct LayerInfo {
        std::string name = "unnamed";
        std::string uuid = "none";
        std::string type = "none";
        std::vector<std::string> can_accept;
        pigment::RGB color;
        concord::Bound bound;
        concord::Polygon field;
        float resolution;
    };

} // namespace fs
