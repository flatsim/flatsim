#include "flatsim/communication/messages.hpp"

namespace fs::messages {

    // PositionMessage serialization
    std::string PositionMessage::serialize() const {
        boost::json::object j;
        j["sender_uuid"] = sender_uuid;
        j["timestamp"] = timestamp;
        j["pose_x"] = pose.point.x;
        j["pose_y"] = pose.point.y;
        j["pose_z"] = pose.point.z;
        j["pose_yaw"] = pose.angle.yaw;
        j["pose_pitch"] = pose.angle.pitch;
        j["pose_roll"] = pose.angle.roll;
        return boost::json::serialize(j);
    }

    PositionMessage PositionMessage::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        PositionMessage msg;
        msg.sender_uuid = boost::json::value_to<std::string>(j.at("sender_uuid"));
        msg.timestamp = boost::json::value_to<double>(j.at("timestamp"));
        msg.pose.point.x = boost::json::value_to<double>(j.at("pose_x"));
        msg.pose.point.y = boost::json::value_to<double>(j.at("pose_y"));
        msg.pose.point.z = boost::json::value_to<double>(j.at("pose_z"));
        msg.pose.angle.yaw = boost::json::value_to<double>(j.at("pose_yaw"));
        msg.pose.angle.pitch = boost::json::value_to<double>(j.at("pose_pitch"));
        msg.pose.angle.roll = boost::json::value_to<double>(j.at("pose_roll"));
        return msg;
    }

    // ControlCommand serialization
    std::string ControlCommand::serialize() const {
        boost::json::object j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        j["steering"] = steering;
        j["throttle"] = throttle;
        return boost::json::serialize(j);
    }

    ControlCommand ControlCommand::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        ControlCommand msg;
        msg.robot_uuid = boost::json::value_to<std::string>(j.at("robot_uuid"));
        msg.timestamp = boost::json::value_to<double>(j.at("timestamp"));
        msg.steering = boost::json::value_to<float>(j.at("steering"));
        msg.throttle = boost::json::value_to<float>(j.at("throttle"));
        return msg;
    }

    // PhysicsState serialization
    std::string PhysicsState::serialize() const {
        boost::json::object j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        j["pose_x"] = pose.point.x;
        j["pose_y"] = pose.point.y;
        j["pose_z"] = pose.point.z;
        j["pose_yaw"] = pose.angle.yaw;
        j["pose_pitch"] = pose.angle.pitch;
        j["pose_roll"] = pose.angle.roll;
        j["velocity_linear"] = velocity.linear;
        j["velocity_angular"] = velocity.angular;
        return boost::json::serialize(j);
    }

    PhysicsState PhysicsState::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        PhysicsState msg;
        msg.robot_uuid = boost::json::value_to<std::string>(j.at("robot_uuid"));
        msg.timestamp = boost::json::value_to<double>(j.at("timestamp"));
        msg.pose.point.x = boost::json::value_to<double>(j.at("pose_x"));
        msg.pose.point.y = boost::json::value_to<double>(j.at("pose_y"));
        msg.pose.point.z = boost::json::value_to<double>(j.at("pose_z"));
        msg.pose.angle.yaw = boost::json::value_to<double>(j.at("pose_yaw"));
        msg.pose.angle.pitch = boost::json::value_to<double>(j.at("pose_pitch"));
        msg.pose.angle.roll = boost::json::value_to<double>(j.at("pose_roll"));
        msg.velocity.linear = boost::json::value_to<float>(j.at("velocity_linear"));
        msg.velocity.angular = boost::json::value_to<float>(j.at("velocity_angular"));
        return msg;
    }

    // RegistrationRequest serialization
    std::string RegistrationRequest::serialize() const {
        boost::json::object j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        return boost::json::serialize(j);
    }

    RegistrationRequest RegistrationRequest::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        RegistrationRequest msg;
        msg.robot_uuid = boost::json::value_to<std::string>(j.at("robot_uuid"));
        msg.timestamp = boost::json::value_to<double>(j.at("timestamp"));
        return msg;
    }

    // RegistrationReply serialization
    std::string RegistrationReply::serialize() const {
        boost::json::object j;
        j["robot_uuid"] = robot_uuid;
        j["assigned_endpoint"] = assigned_endpoint;
        j["success"] = success;
        return boost::json::serialize(j);
    }

    RegistrationReply RegistrationReply::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        RegistrationReply msg;
        msg.robot_uuid = boost::json::value_to<std::string>(j.at("robot_uuid"));
        msg.assigned_endpoint = boost::json::value_to<std::string>(j.at("assigned_endpoint"));
        msg.success = boost::json::value_to<bool>(j.at("success"));
        return msg;
    }

    // SpawnRobotReply serialization
    std::string SpawnRobotReply::serialize() const {
        boost::json::object j;
        j["robot_uuid"] = robot_uuid;
        j["success"] = success;
        j["error_message"] = error_message;
        j["command_endpoint"] = command_endpoint;
        j["state_endpoint"] = state_endpoint;
        return boost::json::serialize(j);
    }

    SpawnRobotReply SpawnRobotReply::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        SpawnRobotReply msg;
        msg.robot_uuid = boost::json::value_to<std::string>(j.at("robot_uuid"));
        msg.success = boost::json::value_to<bool>(j.at("success"));
        msg.error_message = boost::json::value_to<std::string>(j.at("error_message"));
        msg.command_endpoint = boost::json::value_to<std::string>(j.at("command_endpoint"));
        msg.state_endpoint = boost::json::value_to<std::string>(j.at("state_endpoint"));
        return msg;
    }

    // Helper to serialize concord types
    static boost::json::object serialize_point(const concord::Point &p) {
        boost::json::object obj;
        obj["x"] = p.x;
        obj["y"] = p.y;
        obj["z"] = p.z;
        return obj;
    }

    static concord::Point deserialize_point(const boost::json::object &obj) {
        concord::Point p;
        p.x = boost::json::value_to<double>(obj.at("x"));
        p.y = boost::json::value_to<double>(obj.at("y"));
        p.z = boost::json::value_to<double>(obj.at("z"));
        return p;
    }

    static boost::json::object serialize_euler(const concord::Euler &e) {
        boost::json::object obj;
        obj["roll"] = e.roll;
        obj["pitch"] = e.pitch;
        obj["yaw"] = e.yaw;
        return obj;
    }

    static concord::Euler deserialize_euler(const boost::json::object &obj) {
        concord::Euler e;
        e.roll = boost::json::value_to<double>(obj.at("roll"));
        e.pitch = boost::json::value_to<double>(obj.at("pitch"));
        e.yaw = boost::json::value_to<double>(obj.at("yaw"));
        return e;
    }

    static boost::json::object serialize_pose(const concord::Pose &p) {
        boost::json::object obj;
        obj["point"] = serialize_point(p.point);
        obj["angle"] = serialize_euler(p.angle);
        return obj;
    }

    static concord::Pose deserialize_pose(const boost::json::object &obj) {
        concord::Pose p;
        p.point = deserialize_point(obj.at("point").as_object());
        p.angle = deserialize_euler(obj.at("angle").as_object());
        return p;
    }

    static boost::json::object serialize_size(const concord::Size &s) {
        boost::json::object obj;
        obj["x"] = s.x;
        obj["y"] = s.y;
        obj["z"] = s.z;
        return obj;
    }

    static concord::Size deserialize_size(const boost::json::object &obj) {
        concord::Size s;
        s.x = boost::json::value_to<float>(obj.at("x"));
        s.y = boost::json::value_to<float>(obj.at("y"));
        s.z = boost::json::value_to<float>(obj.at("z"));
        return s;
    }

    static boost::json::object serialize_bound(const concord::Bound &b) {
        boost::json::object obj;
        obj["pose"] = serialize_pose(b.pose);
        obj["size"] = serialize_size(b.size);
        return obj;
    }

    static concord::Bound deserialize_bound(const boost::json::object &obj) {
        return concord::Bound(deserialize_pose(obj.at("pose").as_object()),
                              deserialize_size(obj.at("size").as_object()));
    }

    std::string RobotInfoMessage::serialize() const {
        boost::json::object j;
        j["RCI"] = RCI;
        j["group"] = group;
        j["slave"] = slave;
        j["name"] = name;
        j["uuid"] = uuid;
        j["type"] = type;
        j["seqid"] = seqid;

        boost::json::array works_on_arr;
        for (const auto &w : works_on) {
            works_on_arr.push_back(boost::json::string(w));
        }
        j["works_on"] = works_on_arr;

        boost::json::object cap;
        boost::json::array work_on_arr;
        for (const auto &w : capability.work_on) work_on_arr.push_back(boost::json::string(w));
        cap["work_on"] = work_on_arr;

        boost::json::array connect_to_arr;
        for (const auto &c : capability.connect_to) connect_to_arr.push_back(boost::json::string(c));
        cap["connect_to"] = connect_to_arr;

        boost::json::array unload_to_arr;
        for (const auto &u : capability.unload_to) unload_to_arr.push_back(boost::json::string(u));
        cap["unload_to"] = unload_to_arr;
        j["capability"] = cap;

        boost::json::object color_obj;
        color_obj["r"] = color.r;
        color_obj["g"] = color.g;
        color_obj["b"] = color.b;
        j["color"] = color_obj;

        j["bound"] = serialize_bound(bound);
        j["outline"] = boost::json::object(); // Polygon is complex, skip for now

        boost::json::array wheels_arr;
        for (const auto &w : wheels) {
            wheels_arr.push_back(serialize_bound(w));
        }
        j["wheels"] = wheels_arr;

        boost::json::object controls_obj;
        boost::json::array st_max, st_diff, th_max, th_diff, left;
        for (auto v : controls.steerings_max) st_max.push_back(v);
        for (auto v : controls.steerings_diff) st_diff.push_back(v);
        for (auto v : controls.throttles_max) th_max.push_back(v);
        for (auto v : controls.throttles_diff) th_diff.push_back(v);
        for (auto v : controls.left_side) left.push_back(v);
        controls_obj["steerings_max"] = st_max;
        controls_obj["steerings_diff"] = st_diff;
        controls_obj["throttles_max"] = th_max;
        controls_obj["throttles_diff"] = th_diff;
        controls_obj["left_side"] = left;
        j["controls"] = controls_obj;

        boost::json::object hitches_obj;
        for (const auto &[name, hitch] : hitches) {
            boost::json::object h;
            h["bound"] = serialize_bound(hitch.bound);
            h["is_master"] = hitch.is_master;
            hitches_obj[name] = h;
        }
        j["hitches"] = hitches_obj;

        boost::json::array karos_arr;
        for (const auto &k : karos) {
            boost::json::object karo;
            karo["name"] = k.name;
            karo["bound"] = serialize_bound(k.bound);
            boost::json::object kcolor;
            kcolor["r"] = k.color.r;
            kcolor["g"] = k.color.g;
            kcolor["b"] = k.color.b;
            karo["color"] = kcolor;
            karo["sections"] = k.sections;
            karo["has_physics"] = k.has_physics;
            karos_arr.push_back(karo);
        }
        j["karos"] = karos_arr;

        if (tank.has_value()) {
            boost::json::object t;
            t["name"] = tank->name;
            t["capacity"] = tank->capacity;
            t["bound"] = serialize_bound(tank->bound);
            j["tank"] = t;
        }

        if (power_source.has_value()) {
            boost::json::object p;
            p["name"] = power_source->name;
            p["type"] = static_cast<int>(power_source->type);
            p["capacity"] = power_source->capacity;
            p["consumption_rate"] = power_source->consumption_rate;
            p["charge_rate"] = power_source->charge_rate;
            j["power_source"] = p;
        }

        j["role"] = static_cast<int>(role);
        j["turning_radius"] = turning_radius;

        return boost::json::serialize(j);
    }

    RobotInfoMessage RobotInfoMessage::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        RobotInfoMessage msg;
        msg.RCI = boost::json::value_to<uint>(j.at("RCI"));
        msg.group = boost::json::value_to<uint>(j.at("group"));
        msg.slave = boost::json::value_to<bool>(j.at("slave"));
        msg.name = boost::json::value_to<std::string>(j.at("name"));
        msg.uuid = boost::json::value_to<std::string>(j.at("uuid"));
        msg.type = boost::json::value_to<std::string>(j.at("type"));
        msg.seqid = boost::json::value_to<std::string>(j.at("seqid"));

        for (const auto &w : j.at("works_on").as_array()) {
            msg.works_on.push_back(boost::json::value_to<std::string>(w));
        }

        auto const &cap = j.at("capability").as_object();
        for (const auto &w : cap.at("work_on").as_array()) {
            msg.capability.work_on.push_back(boost::json::value_to<std::string>(w));
        }
        for (const auto &c : cap.at("connect_to").as_array()) {
            msg.capability.connect_to.push_back(boost::json::value_to<std::string>(c));
        }
        for (const auto &u : cap.at("unload_to").as_array()) {
            msg.capability.unload_to.push_back(boost::json::value_to<std::string>(u));
        }

        auto const &color_obj = j.at("color").as_object();
        msg.color.r = boost::json::value_to<int>(color_obj.at("r"));
        msg.color.g = boost::json::value_to<int>(color_obj.at("g"));
        msg.color.b = boost::json::value_to<int>(color_obj.at("b"));

        msg.bound = deserialize_bound(j.at("bound").as_object());

        for (const auto &w : j.at("wheels").as_array()) {
            msg.wheels.push_back(deserialize_bound(w.as_object()));
        }

        auto const &controls_obj = j.at("controls").as_object();
        for (const auto &v : controls_obj.at("steerings_max").as_array()) {
            msg.controls.steerings_max.push_back(boost::json::value_to<float>(v));
        }
        for (const auto &v : controls_obj.at("steerings_diff").as_array()) {
            msg.controls.steerings_diff.push_back(boost::json::value_to<float>(v));
        }
        for (const auto &v : controls_obj.at("throttles_max").as_array()) {
            msg.controls.throttles_max.push_back(boost::json::value_to<float>(v));
        }
        for (const auto &v : controls_obj.at("throttles_diff").as_array()) {
            msg.controls.throttles_diff.push_back(boost::json::value_to<float>(v));
        }
        for (const auto &v : controls_obj.at("left_side").as_array()) {
            msg.controls.left_side.push_back(boost::json::value_to<bool>(v));
        }

        auto const &hitches_obj = j.at("hitches").as_object();
        for (auto const &item : hitches_obj) {
            std::string hname = std::string(item.key());
            auto const &h = item.value().as_object();
            HitchInfo hitch;
            hitch.bound = deserialize_bound(h.at("bound").as_object());
            hitch.is_master = boost::json::value_to<bool>(h.at("is_master"));
            msg.hitches[hname] = hitch;
        }

        for (const auto &kv : j.at("karos").as_array()) {
            auto const &karo = kv.as_object();
            KarosserieInfo k;
            k.name = boost::json::value_to<std::string>(karo.at("name"));
            k.bound = deserialize_bound(karo.at("bound").as_object());
            auto const &kcolor = karo.at("color").as_object();
            k.color.r = boost::json::value_to<int>(kcolor.at("r"));
            k.color.g = boost::json::value_to<int>(kcolor.at("g"));
            k.color.b = boost::json::value_to<int>(kcolor.at("b"));
            k.sections = boost::json::value_to<int>(karo.at("sections"));
            k.has_physics = boost::json::value_to<bool>(karo.at("has_physics"));
            msg.karos.push_back(k);
        }

        if (j.contains("tank") && !j.at("tank").is_null()) {
            auto const &t = j.at("tank").as_object();
            TankInfo tank;
            tank.name = boost::json::value_to<std::string>(t.at("name"));
            tank.capacity = boost::json::value_to<float>(t.at("capacity"));
            tank.bound = deserialize_bound(t.at("bound").as_object());
            msg.tank = tank;
        }

        if (j.contains("power_source") && !j.at("power_source").is_null()) {
            auto const &p = j.at("power_source").as_object();
            PowerInfo power;
            power.name = boost::json::value_to<std::string>(p.at("name"));
            power.type = static_cast<PowerType>(boost::json::value_to<int>(p.at("type")));
            power.capacity = boost::json::value_to<float>(p.at("capacity"));
            power.consumption_rate = boost::json::value_to<float>(p.at("consumption_rate"));
            power.charge_rate = boost::json::value_to<float>(p.at("charge_rate"));
            msg.power_source = power;
        }

        msg.role = static_cast<RobotRole>(boost::json::value_to<int>(j.at("role")));
        msg.turning_radius = boost::json::value_to<float>(j.at("turning_radius"));

        return msg;
    }

    std::string SpawnRobotRequest::serialize() const {
        boost::json::object j;
        j["robot_info"] = robot_info.serialize();
        j["timestamp"] = timestamp;
        j["use_tcp"] = use_tcp;
        return boost::json::serialize(j);
    }

    SpawnRobotRequest SpawnRobotRequest::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        SpawnRobotRequest msg;
        msg.robot_info = RobotInfoMessage::deserialize(boost::json::value_to<std::string>(j.at("robot_info")));
        msg.timestamp = boost::json::value_to<double>(j.at("timestamp"));
        // Handle both old bool format and new string format for backward compatibility
        if (j.contains("use_tcp")) {
            if (j.at("use_tcp").is_bool()) {
                // Old format: convert bool to empty string (IPC) or "127.0.0.1" (TCP fallback)
                msg.use_tcp = boost::json::value_to<bool>(j.at("use_tcp")) ? "127.0.0.1" : "";
            } else {
                msg.use_tcp = boost::json::value_to<std::string>(j.at("use_tcp"));
            }
        } else {
            msg.use_tcp = ""; // Default to IPC
        }
        return msg;
    }

    // HeartbeatMessage serialization
    std::string HeartbeatMessage::serialize() const {
        boost::json::object j;
        j["robot_uuid"] = robot_uuid;
        j["timestamp"] = timestamp;
        return boost::json::serialize(j);
    }

    HeartbeatMessage HeartbeatMessage::deserialize(const std::string &data) {
        boost::json::value jv = boost::json::parse(data);
        boost::json::object const &j = jv.as_object();

        HeartbeatMessage msg;
        msg.robot_uuid = boost::json::value_to<std::string>(j.at("robot_uuid"));
        msg.timestamp = boost::json::value_to<double>(j.at("timestamp"));
        return msg;
    }

} // namespace fs::messages
