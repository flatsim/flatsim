#pragma once

#include "flatsim/types.hpp"
#include <nlohmann/json.hpp>
#include <string>

namespace fs::messages {

    // Basic position update message for robot-to-robot communication
    struct PositionMessage {
        std::string sender_uuid;
        double timestamp;
        concord::Pose pose;

        // Default constructor
        PositionMessage() = default;

        // Constructor
        PositionMessage(const std::string &uuid, double ts, const concord::Pose &p)
            : sender_uuid(uuid), timestamp(ts), pose(p) {}

        // Serialization
        std::string serialize() const;
        static PositionMessage deserialize(const std::string &data);
    };

    // Control command message (for future process separation)
    struct ControlCommand {
        std::string robot_uuid;
        double timestamp;
        float steering;
        float throttle;

        // Default constructor
        ControlCommand() = default;

        // Constructor
        ControlCommand(const std::string &uuid, double ts, float s, float t)
            : robot_uuid(uuid), timestamp(ts), steering(s), throttle(t) {}

        // Serialization
        std::string serialize() const;
        static ControlCommand deserialize(const std::string &data);
    };

    // Physics state message (for future process separation)
    struct PhysicsState {
        std::string robot_uuid;
        double timestamp;
        concord::Pose pose;
        struct {
            float linear;
            float angular;
        } velocity;

        // Default constructor
        PhysicsState() = default;

        // Constructor
        PhysicsState(const std::string &uuid, double ts, const concord::Pose &p)
            : robot_uuid(uuid), timestamp(ts), pose(p) {
            velocity.linear = 0.0f;
            velocity.angular = 0.0f;
        }

        // Serialization
        std::string serialize() const;
        static PhysicsState deserialize(const std::string &data);
    };

    // Registration messages for robot-simulator handshake
    struct RegistrationRequest {
        std::string robot_uuid;
        double timestamp;

        // Default constructor
        RegistrationRequest() = default;

        // Constructor
        RegistrationRequest(const std::string &uuid, double ts) : robot_uuid(uuid), timestamp(ts) {}

        // Serialization
        std::string serialize() const;
        static RegistrationRequest deserialize(const std::string &data);
    };

    struct RegistrationReply {
        std::string robot_uuid;
        std::string assigned_endpoint;
        bool success;

        // Default constructor
        RegistrationReply() : success(false) {}

        // Constructor
        RegistrationReply(const std::string &uuid, const std::string &endpoint, bool s)
            : robot_uuid(uuid), assigned_endpoint(endpoint), success(s) {}

        // Serialization
        std::string serialize() const;
        static RegistrationReply deserialize(const std::string &data);
    };

    // RobotInfo message for serialization/deserialization
    struct RobotInfoMessage {
        uint RCI;
        uint group;
        bool slave = false;
        std::string name = "unnamed";
        std::string uuid = "none";
        std::string type = "none";
        std::string seqid = "none";
        std::vector<std::string> works_on;
        Capability capability;
        pigment::RGB color;
        concord::Bound bound;
        concord::Polygon outline;
        std::vector<concord::Bound> wheels;
        RobotControll controls;
        std::unordered_map<std::string, HitchInfo> hitches;
        std::vector<KarosserieInfo> karos;
        std::optional<TankInfo> tank;
        std::optional<PowerInfo> power_source;
        RobotRole role = RobotRole::MASTER;
        float turning_radius = 1.0f;

        // Default constructor
        RobotInfoMessage() = default;

        // Constructor from RobotInfo
        RobotInfoMessage(const RobotInfo &info)
            : RCI(info.RCI), group(info.group), slave(info.slave), name(info.name), uuid(info.uuid), type(info.type),
              seqid(info.seqid), works_on(info.works_on), capability(info.capability), color(info.color),
              bound(info.bound), outline(info.outline), wheels(info.wheels), controls(info.controls),
              hitches(info.hitches), karos(info.karos), tank(info.tank), power_source(info.power_source),
              role(info.role), turning_radius(info.turning_radius) {}

        // Convert to RobotInfo
        RobotInfo to_robot_info() const {
            RobotInfo info;
            info.RCI = RCI;
            info.group = group;
            info.slave = slave;
            info.name = name;
            info.uuid = uuid;
            info.type = type;
            info.seqid = seqid;
            info.works_on = works_on;
            info.capability = capability;
            info.color = color;
            info.bound = bound;
            info.outline = outline;
            info.wheels = wheels;
            info.controls = controls;
            info.hitches = hitches;
            info.karos = karos;
            info.tank = tank;
            info.power_source = power_source;
            info.role = role;
            info.turning_radius = turning_radius;
            return info;
        }

        // Serialization
        std::string serialize() const;
        static RobotInfoMessage deserialize(const std::string &data);
    };

    // Spawn robot messages for process separation
    struct SpawnRobotRequest {
        RobotInfoMessage robot_info;
        double timestamp;

        // Default constructor
        SpawnRobotRequest() = default;

        // Constructor
        SpawnRobotRequest(const RobotInfoMessage &info, double ts) : robot_info(info), timestamp(ts) {}

        // Serialization
        std::string serialize() const;
        static SpawnRobotRequest deserialize(const std::string &data);
    };

    struct SpawnRobotReply {
        std::string robot_uuid;
        bool success;
        std::string error_message;

        // Default constructor
        SpawnRobotReply() : success(false) {}

        // Constructor
        SpawnRobotReply(const std::string &uuid, bool s, const std::string &err = "")
            : robot_uuid(uuid), success(s), error_message(err) {}

        // Serialization
        std::string serialize() const;
        static SpawnRobotReply deserialize(const std::string &data);
    };

} // namespace fs::messages

// JSON serialization for all RobotInfoMessage subtypes
namespace nlohmann {
    // pigment::RGB
    template <> struct adl_serializer<pigment::RGB> {
        static void to_json(json &j, const pigment::RGB &rgb) { j = json{{"r", rgb.r}, {"g", rgb.g}, {"b", rgb.b}}; }
        static void from_json(const json &j, pigment::RGB &rgb) {
            rgb.r = j.value("r", 0);
            rgb.g = j.value("g", 0);
            rgb.b = j.value("b", 0);
        }
    };

    // concord::Point
    template <> struct adl_serializer<concord::Point> {
        static void to_json(json &j, const concord::Point &p) { j = json{{"x", p.x}, {"y", p.y}, {"z", p.z}}; }
        static void from_json(const json &j, concord::Point &p) {
            p.x = j.value("x", 0.0);
            p.y = j.value("y", 0.0);
            p.z = j.value("z", 0.0);
        }
    };

    // concord::Euler (not Angle)
    template <> struct adl_serializer<concord::Euler> {
        static void to_json(json &j, const concord::Euler &e) {
            j = json{{"roll", e.roll}, {"pitch", e.pitch}, {"yaw", e.yaw}};
        }
        static void from_json(const json &j, concord::Euler &e) {
            e.roll = j.value("roll", 0.0);
            e.pitch = j.value("pitch", 0.0);
            e.yaw = j.value("yaw", 0.0);
        }
    };

    // concord::Pose
    template <> struct adl_serializer<concord::Pose> {
        static void to_json(json &j, const concord::Pose &p) { j = json{{"point", p.point}, {"angle", p.angle}}; }
        static void from_json(const json &j, concord::Pose &p) {
            p.point = j.value("point", concord::Point{0.0, 0.0, 0.0});
            p.angle = j.value("angle", concord::Euler{0.0, 0.0, 0.0});
        }
    };

    // concord::Size
    template <> struct adl_serializer<concord::Size> {
        static void to_json(json &j, const concord::Size &s) { j = json{{"x", s.x}, {"y", s.y}, {"z", s.z}}; }
        static void from_json(const json &j, concord::Size &s) {
            s.x = j.value("x", 0.0f);
            s.y = j.value("y", 0.0f);
            s.z = j.value("z", 0.0f);
        }
    };

    // concord::Bound
    template <> struct adl_serializer<concord::Bound> {
        static void to_json(json &j, const concord::Bound &b) { j = json{{"pose", b.pose}, {"size", b.size}}; }
        static void from_json(const json &j, concord::Bound &b) {
            b.pose = j.value("pose", concord::Pose{concord::Point{0.0, 0.0, 0.0}, concord::Euler{0.0, 0.0, 0.0}});
            b.size = j.value("size", concord::Size{0.0f, 0.0f, 0.0f});
        }
    };

    // concord::Polygon - simplified since points is private
    template <> struct adl_serializer<concord::Polygon> {
        static void to_json(json &j, const concord::Polygon &poly) { j = json{{"vertex_count", poly.numVertices()}}; }
        static void from_json(const json &j, concord::Polygon &poly) {
            // Can't access private points, so just skip for now
            auto vertex_count = j.value("vertex_count", 0);
            (void)vertex_count; // suppress unused variable warning
        }
    };

    // fs::Capability
    template <> struct adl_serializer<fs::Capability> {
        static void to_json(json &j, const fs::Capability &cap) {
            j = json{{"work_on", cap.work_on}, {"connect_to", cap.connect_to}, {"unload_to", cap.unload_to}};
        }
        static void from_json(const json &j, fs::Capability &cap) {
            cap.work_on = j.value("work_on", std::vector<std::string>{});
            cap.connect_to = j.value("connect_to", std::vector<std::string>{});
            cap.unload_to = j.value("unload_to", std::vector<std::string>{});
        }
    };

    // fs::RobotControll
    template <> struct adl_serializer<fs::RobotControll> {
        static void to_json(json &j, const fs::RobotControll &ctrl) {
            j = json{{"steerings_max", ctrl.steerings_max},
                     {"throttles_max", ctrl.throttles_max},
                     {"steerings_diff", ctrl.steerings_diff},
                     {"throttles_diff", ctrl.throttles_diff},
                     {"left_side", ctrl.left_side}};
        }
        static void from_json(const json &j, fs::RobotControll &ctrl) {
            ctrl.steerings_max = j.value("steerings_max", std::vector<float>{});
            ctrl.throttles_max = j.value("throttles_max", std::vector<float>{});
            ctrl.steerings_diff = j.value("steerings_diff", std::vector<float>{});
            ctrl.throttles_diff = j.value("throttles_diff", std::vector<float>{});
            ctrl.left_side = j.value("left_side", std::vector<bool>{});
        }
    };

    // fs::HitchInfo
    template <> struct adl_serializer<fs::HitchInfo> {
        static void to_json(json &j, const fs::HitchInfo &hitch) {
            j = json{{"bound", hitch.bound}, {"is_master", hitch.is_master}};
        }
        static void from_json(const json &j, fs::HitchInfo &hitch) {
            hitch.bound = j.value(
                "bound", concord::Bound{concord::Pose{concord::Point{0.0, 0.0, 0.0}, concord::Euler{0.0, 0.0, 0.0}},
                                        concord::Size{0.0f, 0.0f, 0.0f}});
            hitch.is_master = j.value("is_master", true);
        }
    };

    // fs::KarosserieInfo
    template <> struct adl_serializer<fs::KarosserieInfo> {
        static void to_json(json &j, const fs::KarosserieInfo &kar) {
            j = json{{"name", kar.name},
                     {"bound", kar.bound},
                     {"color", kar.color},
                     {"sections", kar.sections},
                     {"has_physics", kar.has_physics}};
        }
        static void from_json(const json &j, fs::KarosserieInfo &kar) {
            kar.name = j.value("name", std::string{""});
            kar.bound = j.value(
                "bound", concord::Bound{concord::Pose{concord::Point{0.0, 0.0, 0.0}, concord::Euler{0.0, 0.0, 0.0}},
                                        concord::Size{0.0f, 0.0f, 0.0f}});
            kar.color = j.value("color", pigment::RGB{0, 0, 0});
            kar.sections = j.value("sections", 0);
            kar.has_physics = j.value("has_physics", true);
        }
    };

    // fs::TankInfo
    template <> struct adl_serializer<fs::TankInfo> {
        static void to_json(json &j, const fs::TankInfo &tank) {
            j = json{{"name", tank.name}, {"capacity", tank.capacity}, {"bound", tank.bound}};
        }
        static void from_json(const json &j, fs::TankInfo &tank) {
            tank.name = j.value("name", std::string{""});
            tank.capacity = j.value("capacity", 0.0f);
            tank.bound = j.value(
                "bound", concord::Bound{concord::Pose{concord::Point{0.0, 0.0, 0.0}, concord::Euler{0.0, 0.0, 0.0}},
                                        concord::Size{0.0f, 0.0f, 0.0f}});
        }
    };

    // fs::PowerInfo
    template <> struct adl_serializer<fs::PowerInfo> {
        static void to_json(json &j, const fs::PowerInfo &power) {
            j = json{{"name", power.name},
                     {"type", static_cast<int>(power.type)},
                     {"capacity", power.capacity},
                     {"consumption_rate", power.consumption_rate},
                     {"charge_rate", power.charge_rate}};
        }
        static void from_json(const json &j, fs::PowerInfo &power) {
            power.name = j.value("name", std::string{""});
            power.type = static_cast<fs::PowerType>(j.value("type", 0));
            power.capacity = j.value("capacity", 0.0f);
            power.consumption_rate = j.value("consumption_rate", 0.0f);
            power.charge_rate = j.value("charge_rate", 0.0f);
        }
    };

    // Main RobotInfoMessage serialization
    template <> struct adl_serializer<fs::messages::RobotInfoMessage> {
        static void to_json(json &j, const fs::messages::RobotInfoMessage &info) {
            j = json{{"RCI", info.RCI},
                     {"group", info.group},
                     {"slave", info.slave},
                     {"name", info.name},
                     {"uuid", info.uuid},
                     {"type", info.type},
                     {"seqid", info.seqid},
                     {"works_on", info.works_on},
                     {"capability", info.capability},
                     {"color", info.color},
                     {"bound", info.bound},
                     {"outline", info.outline},
                     {"wheels", info.wheels},
                     {"controls", info.controls},
                     {"hitches", info.hitches},
                     {"karos", info.karos},
                     {"tank", info.tank},
                     {"power_source", info.power_source},
                     {"role", static_cast<int>(info.role)},
                     {"turning_radius", info.turning_radius}};
        }

        static void from_json(const json &j, fs::messages::RobotInfoMessage &info) {
            info.RCI = j.value("RCI", 0u);
            info.group = j.value("group", 0u);
            info.slave = j.value("slave", false);
            info.name = j.value("name", "unnamed");
            info.uuid = j.value("uuid", "none");
            info.type = j.value("type", "none");
            info.seqid = j.value("seqid", "none");
            info.works_on = j.value("works_on", std::vector<std::string>{});
            info.capability = j.value("capability", fs::Capability{});
            info.color = j.value("color", pigment::RGB{0, 0, 0});
            info.bound = j.value(
                "bound", concord::Bound{concord::Pose{concord::Point{0.0, 0.0, 0.0}, concord::Euler{0.0, 0.0, 0.0}},
                                        concord::Size{0.0f, 0.0f, 0.0f}});
            info.outline = j.value("outline", concord::Polygon{});
            info.wheels = j.value("wheels", std::vector<concord::Bound>{});
            info.controls = j.value("controls", fs::RobotControll{});
            info.hitches = j.value("hitches", std::unordered_map<std::string, fs::HitchInfo>{});
            info.karos = j.value("karos", std::vector<fs::KarosserieInfo>{});

            // Handle optional fields properly
            if (j.contains("tank") && !j["tank"].is_null()) {
                info.tank = j["tank"].get<fs::TankInfo>();
            } else {
                info.tank = std::nullopt;
            }

            if (j.contains("power_source") && !j["power_source"].is_null()) {
                info.power_source = j["power_source"].get<fs::PowerInfo>();
            } else {
                info.power_source = std::nullopt;
            }

            info.role = static_cast<fs::RobotRole>(j.value("role", 0));
            info.turning_radius = j.value("turning_radius", 1.0f);
        }
    };
} // namespace nlohmann