#include "flatsim/agent/logging_agent.hpp"

namespace fs::agent {

    LoggingAgent::LoggingAgent(const protocol::RobotId &id, std::shared_ptr<rerun::RecordingStream> rec)
        : id_(id), rec_(std::move(rec)) {}

    void LoggingAgent::on_state(const protocol::RobotState &state, float /*dt*/) {
        last_state_ = state;

        if (!rec_) return;

        const auto &p = state.pose.point;
        auto color = rerun::Color(0, 255, 0);

        rec_->log_static(id_ + "/pose",
                         rerun::Points3D({{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)}})
                             .with_colors({color}));
    }

    protocol::RobotCommand LoggingAgent::compute_command() {
        protocol::RobotCommand cmd;
        cmd.id = id_;
        cmd.timestamp = last_state_.timestamp;
        cmd.steering = 0.0f;
        cmd.throttle = 0.0f;
        return cmd;
    }

} // namespace fs::agent
