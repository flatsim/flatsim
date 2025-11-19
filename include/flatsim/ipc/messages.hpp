#pragma once

#include "flatsim/robot/types.hpp"
#include <boost/json.hpp>
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
        std::string use_tcp = ""; // Empty = IPC, non-empty = TCP with client's connected IP

        // Default constructor
        SpawnRobotRequest() = default;

        // Constructor
        SpawnRobotRequest(const RobotInfoMessage &info, double ts, const std::string &tcp_ip = "")
            : robot_info(info), timestamp(ts), use_tcp(tcp_ip) {}

        // Serialization
        std::string serialize() const;
        static SpawnRobotRequest deserialize(const std::string &data);
    };

    struct SpawnRobotReply {
        std::string robot_uuid;
        bool success;
        std::string error_message;
        std::string command_endpoint;
        std::string state_endpoint;

        // Default constructor
        SpawnRobotReply() : success(false) {}

        // Constructor
        SpawnRobotReply(const std::string &uuid, bool s, const std::string &err = "")
            : robot_uuid(uuid), success(s), error_message(err) {}

        // Serialization
        std::string serialize() const;
        static SpawnRobotReply deserialize(const std::string &data);
    };

    // Heartbeat message for robot online status tracking
    struct HeartbeatMessage {
        std::string robot_uuid;
        double timestamp;

        // Default constructor
        HeartbeatMessage() = default;

        // Constructor
        HeartbeatMessage(const std::string &uuid, double ts) : robot_uuid(uuid), timestamp(ts) {}

        // Serialization
        std::string serialize() const;
        static HeartbeatMessage deserialize(const std::string &data);
    };

} // namespace fs::messages
