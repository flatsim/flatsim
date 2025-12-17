#include "flatsim/ipc/adapters.hpp"

namespace fs::ipc {

    protocol::RobotState to_protocol(const messages::PhysicsState &state) {
        protocol::RobotState out;
        out.id = state.robot_uuid;
        out.timestamp = state.timestamp;
        out.pose = state.pose;
        out.velocity.linear = static_cast<double>(state.velocity.linear);
        out.velocity.angular = static_cast<double>(state.velocity.angular);
        return out;
    }

    messages::PhysicsState from_protocol(const protocol::RobotState &state) {
        messages::PhysicsState out(state.id, state.timestamp, state.pose);
        out.velocity.linear = static_cast<float>(state.velocity.linear);
        out.velocity.angular = static_cast<float>(state.velocity.angular);
        return out;
    }

    protocol::RobotCommand to_protocol(const messages::ControlCommand &cmd) {
        protocol::RobotCommand out;
        out.id = cmd.robot_uuid;
        out.timestamp = cmd.timestamp;
        out.steering = cmd.steering;
        out.throttle = cmd.throttle;
        return out;
    }

    messages::ControlCommand from_protocol(const protocol::RobotCommand &cmd) {
        messages::ControlCommand out(cmd.id, cmd.timestamp, cmd.steering, cmd.throttle);
        return out;
    }

} // namespace fs::ipc
