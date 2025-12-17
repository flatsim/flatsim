#include "flatsim/agent.hpp"

namespace fs {

    Agent::Agent(const protocol::RobotId &id, const drivekit::RobotConstraints &constraints, drivekit::TrackerType type,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : id_(id), rec_(rec) {
        tracker_ = std::make_unique<drivekit::Tracker>(type);
        tracker_->init(constraints, rec, "agent/" + id);
    }

    void Agent::on_state(const protocol::RobotState &state, float dt) {
        last_dt_ = dt;
        last_state_ = state;

        // Update navigation state
        last_nav_state_.pose = state.pose;
        last_nav_state_.velocity.linear = state.velocity.linear;
        last_nav_state_.velocity.angular = state.velocity.angular;

        // Update tracker
        last_cmd_ = tracker_->tick(last_nav_state_, dt);

        // Log if enabled (additional logging beyond what tracker does).
        // Visualization remains on the simulator/server side.
        if (rec_) {
            log_state(state);
        }
    }

    protocol::RobotCommand Agent::compute_command() {
        // Convert to protocol command
        protocol::RobotCommand cmd;
        cmd.id = id_;
        cmd.steering = last_cmd_.angular_velocity;
        cmd.throttle = last_cmd_.linear_velocity;
        cmd.timestamp = 0.0; // Caller can fill if needed

        return cmd;
    }

    void Agent::set_goal(const drivekit::NavigationGoal &goal) { tracker_->set_goal(goal); }

    void Agent::clear_goal() { tracker_->clear_goal(); }

    bool Agent::is_goal_reached() const { return tracker_->is_goal_reached(); }

    void Agent::enable_logging(std::shared_ptr<rerun::RecordingStream> rec) {
        rec_ = rec;
        // Note: tracker already initialized, can't reinitialize with new rec
        // For now, just store rec for additional logging
    }

    void Agent::disable_logging() { rec_ = nullptr; }

    void Agent::log_state(const protocol::RobotState &state) {
        if (!rec_) return;

        std::string entity_path = "agent/" + id_;

        // Log velocity scalars
        rec_->log(entity_path + "/velocity/linear", rerun::Scalars(state.velocity.linear));
        rec_->log(entity_path + "/velocity/angular", rerun::Scalars(state.velocity.angular));
    }

} // namespace fs
