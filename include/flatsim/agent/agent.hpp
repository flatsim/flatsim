#pragma once

#include "flatsim/protocol/types.hpp"

namespace fs::agent {

    class IRobotAgent {
      public:
        virtual ~IRobotAgent() = default;

        // Called whenever a new state is available.
        virtual void on_state(const protocol::RobotState &state, float dt) = 0;

        // Compute the next command based on the most recent state.
        virtual protocol::RobotCommand compute_command() = 0;
    };

} // namespace fs::agent
