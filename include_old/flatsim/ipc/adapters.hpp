#pragma once

#include "flatsim/ipc/messages.hpp"
#include "flatsim/protocol/types.hpp"

namespace fs::ipc {

    // Physics state <-> protocol::RobotState
    protocol::RobotState to_protocol(const messages::PhysicsState &state);
    messages::PhysicsState from_protocol(const protocol::RobotState &state);

    // Control command <-> protocol::RobotCommand
    protocol::RobotCommand to_protocol(const messages::ControlCommand &cmd);
    messages::ControlCommand from_protocol(const protocol::RobotCommand &cmd);

} // namespace fs::ipc
