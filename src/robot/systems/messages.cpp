#include "flatsim/robot/systems/messages.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace fs::messages {

    // PositionMessage serialization
    std::string PositionMessage::serialize() const {
        json j;
        j["sender_uuid"] = sender_uuid;
        j["timestamp"] = timestamp;
        j["pose"] = pose;
        return j.dump();
    }

    PositionMessage PositionMessage::deserialize(const std::string &data) {
        json j = json::parse(data);
        PositionMessage msg;
        msg.sender_uuid = j.value("sender_uuid", "");
        msg.timestamp = j.value("timestamp", 0.0);
        msg.pose = j.value("pose", concord::Pose{});
        return msg;
    }

    // ControlCommand serialization
    std::string ControlCommand::serialize() const {
        json j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        j["steering"] = steering;
        j["throttle"] = throttle;
        return j.dump();
    }

    ControlCommand ControlCommand::deserialize(const std::string &data) {
        json j = json::parse(data);
        ControlCommand msg;
        msg.robot_uuid = j.value("robot_uuid", "");
        msg.timestamp = j.value("timestamp", 0.0);
        msg.steering = j.value("steering", 0.0f);
        msg.throttle = j.value("throttle", 0.0f);
        return msg;
    }

    // PhysicsState serialization
    std::string PhysicsState::serialize() const {
        json j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        j["pose"] = pose;
        j["velocity_linear"] = velocity.linear;
        j["velocity_angular"] = velocity.angular;
        return j.dump();
    }

    PhysicsState PhysicsState::deserialize(const std::string &data) {
        json j = json::parse(data);
        PhysicsState msg;
        msg.robot_uuid = j.value("robot_uuid", "");
        msg.timestamp = j.value("timestamp", 0.0);
        msg.pose = j.value("pose", concord::Pose{});
        msg.velocity.linear = j.value("velocity_linear", 0.0f);
        msg.velocity.angular = j.value("velocity_angular", 0.0f);
        return msg;
    }

    // RegistrationRequest serialization
    std::string RegistrationRequest::serialize() const {
        json j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        return j.dump();
    }

    RegistrationRequest RegistrationRequest::deserialize(const std::string &data) {
        json j = json::parse(data);
        RegistrationRequest msg;
        msg.robot_uuid = j.value("robot_uuid", "");
        msg.timestamp = j.value("timestamp", 0.0);
        return msg;
    }

    // RegistrationReply serialization
    std::string RegistrationReply::serialize() const {
        json j;
        j["robot_uuid"] = robot_uuid;
        j["assigned_endpoint"] = assigned_endpoint;
        j["success"] = success;
        return j.dump();
    }

    RegistrationReply RegistrationReply::deserialize(const std::string &data) {
        json j = json::parse(data);
        RegistrationReply msg;
        msg.robot_uuid = j.value("robot_uuid", "");
        msg.assigned_endpoint = j.value("assigned_endpoint", "");
        msg.success = j.value("success", false);
        return msg;
    }

    // RobotInfoMessage serialization (uses nlohmann JSON serializer)
    std::string RobotInfoMessage::serialize() const {
        json j = *this;
        return j.dump();
    }

    RobotInfoMessage RobotInfoMessage::deserialize(const std::string &data) {
        json j = json::parse(data);
        return j.get<RobotInfoMessage>();
    }

    // SpawnRobotRequest serialization
    std::string SpawnRobotRequest::serialize() const {
        json j;
        j["robot_info"] = robot_info;
        j["timestamp"] = timestamp;
        return j.dump();
    }

    SpawnRobotRequest SpawnRobotRequest::deserialize(const std::string &data) {
        json j = json::parse(data);
        SpawnRobotRequest msg;
        msg.robot_info = j.value("robot_info", RobotInfoMessage{});
        msg.timestamp = j.value("timestamp", 0.0);
        return msg;
    }

    // SpawnRobotReply serialization
    std::string SpawnRobotReply::serialize() const {
        json j;
        j["robot_uuid"] = robot_uuid;
        j["success"] = success;
        j["error_message"] = error_message;
        return j.dump();
    }

    SpawnRobotReply SpawnRobotReply::deserialize(const std::string &data) {
        json j = json::parse(data);
        SpawnRobotReply msg;
        msg.robot_uuid = j.value("robot_uuid", "");
        msg.success = j.value("success", false);
        msg.error_message = j.value("error_message", "");
        return msg;
    }

} // namespace fs::messages
