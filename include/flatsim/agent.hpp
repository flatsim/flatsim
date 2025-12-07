#pragma once

#include "flatsim/protocol/types.hpp"
#include "navcon.hpp"
#include <memory>
#include <optional>
#include <rerun.hpp>

namespace fs {

    /**
     * @brief Unified robot agent with navigation and optional logging.
     *
     * This is the main agent class that handles:
     * - Navigation (path tracking, goal reaching)
     * - Logging (optional Rerun visualization)
     * - Future: perception, planning, etc.
     */
    class Agent {
      public:
        /**
         * @brief Create agent with navigation capabilities.
         *
         * @param id Robot identifier
         * @param constraints Robot kinematic constraints
         * @param type Navigation controller type (PID, CARROT, MPC, etc.)
         * @param rec Optional Rerun recording stream for logging
         */
        Agent(const protocol::RobotId &id, const navcon::RobotConstraints &constraints,
              navcon::TrackerType type = navcon::TrackerType::PID,
              std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Main agent interface
        void on_state(const protocol::RobotState &state, float dt);
        protocol::RobotCommand compute_command();

        // Navigation control
        void set_goal(const navcon::NavigationGoal &goal);
        void clear_goal();
        bool is_goal_reached() const;

        // Enable/disable logging
        void enable_logging(std::shared_ptr<rerun::RecordingStream> rec);
        void disable_logging();
        bool is_logging_enabled() const { return rec_ != nullptr; }

        // Access to navigation tracker
        navcon::Tracker *tracker() { return tracker_.get(); }
        const navcon::Tracker *tracker() const { return tracker_.get(); }

        // Get agent ID
        const protocol::RobotId &id() const { return id_; }

      private:
        // Identity
        protocol::RobotId id_;

        // Navigation component
        std::unique_ptr<navcon::Tracker> tracker_;
        navcon::RobotState last_nav_state_;
        float last_dt_{0.0f};
        navcon::VelocityCommand last_cmd_{};

        // Logging component (optional)
        std::shared_ptr<rerun::RecordingStream> rec_;
        protocol::RobotState last_state_;

        // Internal helpers
        void log_state(const protocol::RobotState &state);
    };

} // namespace fs
