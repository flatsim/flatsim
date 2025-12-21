#pragma once

#include "flatsim/robot/types.hpp"
#include <string>

namespace fs::messages {

    /**
     * @brief Position message for robot-to-robot communication
     *
     * Used by network interfaces (Zenoh, CAN-bus, WiFi) to share
     * position information between robots in a fleet.
     */
    struct PositionMessage {
        std::string sender_uuid;
        double timestamp;
        concord::Pose pose;

        // Default constructor
        PositionMessage() = default;

        // Constructor
        PositionMessage(const std::string &uuid, double ts, const concord::Pose &p)
            : sender_uuid(uuid), timestamp(ts), pose(p) {}
    };

    /**
     * @brief Control command message for robot-to-robot communication
     *
     * Used by network interfaces to send control commands between robots
     * (e.g., master robot controlling slave robots).
     */
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
    };

} // namespace fs::messages
