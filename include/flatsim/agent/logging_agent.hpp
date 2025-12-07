#pragma once

#include "flatsim/agent/agent.hpp"
#include <rerun.hpp>

namespace fs::agent {

    /**
     * @brief Lightweight agent wrapper that handles Rerun logging for a robot.
     *
     * This is optional and can be composed with other agents: you can forward
     * states into it purely for visualization while another agent computes
     * control commands.
     */
    class LoggingAgent : public IRobotAgent {
      public:
        LoggingAgent(const protocol::RobotId &id, std::shared_ptr<rerun::RecordingStream> rec);

        void on_state(const protocol::RobotState &state, float dt) override;
        protocol::RobotCommand compute_command() override;

      private:
        protocol::RobotId id_;
        std::shared_ptr<rerun::RecordingStream> rec_;
        protocol::RobotState last_state_;
    };

} // namespace fs::agent

