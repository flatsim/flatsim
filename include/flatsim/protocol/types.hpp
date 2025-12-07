#pragma once

#include "flatsim/robot/types.hpp"
#include <string>

namespace fs::protocol {

    using RobotId = std::string;

    struct Velocity {
        double linear{0.0};  // forward velocity in robot frame
        double angular{0.0}; // yaw rate (rad/s)
    };

    struct RobotState {
        RobotId id;
        double timestamp{0.0};
        concord::Pose pose{};
        Velocity velocity{};
    };

    struct RobotCommand {
        RobotId id;
        double timestamp{0.0};
        float steering{0.0f};
        float throttle{0.0f};
    };

    struct CollisionEvent {
        RobotId a;
        RobotId b; // "world" or other id if needed
        double timestamp{0.0};
    };

} // namespace fs::protocol
