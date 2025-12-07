#include "flatsim/agent/nav_agent.hpp"

namespace fs::agent {

    NavAgent::NavAgent(const protocol::RobotId &id, const navcon::RobotConstraints &constraints,
                       navcon::TrackerType type)
        : id_(id) {
        tracker_ = std::make_unique<navcon::Tracker>(type);
        // Initialize tracker without rerun stream; agents can supply one if needed.
        tracker_->init(constraints, nullptr, id_);
    }

    void NavAgent::on_state(const protocol::RobotState &state, float dt) {
        last_dt_ = dt;

        // Map protocol::RobotState -> navcon::RobotState
        last_nav_state_.pose = state.pose;
        last_nav_state_.velocity.linear = state.velocity.linear;
        last_nav_state_.velocity.angular = state.velocity.angular;
        last_nav_state_.timestamp = state.timestamp;

        // Compute new velocity command immediately and cache it
        last_cmd_ = tracker_->tick(last_nav_state_, dt);
    }

    protocol::RobotCommand NavAgent::compute_command() {
        protocol::RobotCommand cmd;
        cmd.id = id_;
        cmd.timestamp = last_nav_state_.timestamp;

        if (last_cmd_.valid) {
            cmd.throttle = static_cast<float>(last_cmd_.linear_velocity);
            // Note: flatsim Robot uses opposite angular convention (CW vs CCW),
            // but that inversion is applied on the simulator side; agents
            // operate in navcon's convention here.
            cmd.steering = static_cast<float>(last_cmd_.angular_velocity);
        } else {
            cmd.throttle = 0.0f;
            cmd.steering = 0.0f;
        }

        return cmd;
    }

    void NavAgent::set_goal(const navcon::NavigationGoal &goal) { tracker_->set_goal(goal); }

    void NavAgent::clear_goal() { tracker_->clear_goal(); }

} // namespace fs::agent
