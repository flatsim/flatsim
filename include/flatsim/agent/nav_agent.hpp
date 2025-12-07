#pragma once

#include "flatsim/agent/agent.hpp"
#include "navcon.hpp"

namespace fs::agent {

    /**
     * @brief Simple navcon-based agent operating on protocol::RobotState.
     *
     * This is intentionally minimal: it exposes enough hooks to be useful
     * in both in-process and client/server setups without pulling in
     * simulator internals.
     */
    class NavAgent : public IRobotAgent {
      public:
        explicit NavAgent(const protocol::RobotId &id, const navcon::RobotConstraints &constraints,
                          navcon::TrackerType type = navcon::TrackerType::PID);

        void on_state(const protocol::RobotState &state, float dt) override;
        protocol::RobotCommand compute_command() override;

        // Goal management helpers
        void set_goal(const navcon::NavigationGoal &goal);
        void clear_goal();

        // Access underlying tracker if needed
        navcon::Tracker *tracker() { return tracker_.get(); }
        const navcon::Tracker *tracker() const { return tracker_.get(); }

      private:
        protocol::RobotId id_;
        std::unique_ptr<navcon::Tracker> tracker_;
        navcon::RobotState last_nav_state_;
        float last_dt_{0.0f};
        navcon::VelocityCommand last_cmd_{};
    };

} // namespace fs::agent

