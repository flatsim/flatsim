#pragma once

#include <memory>
#include <string>

#include "flatsim/agent/control_manager.hpp"
#include "flatsim/types.hpp"
#include <rerun.hpp>

namespace agent {

    class Machine {
      private:
        std::shared_ptr<rerun::RecordingStream> rec_;

        types::Machine config_;
        types::State state_;
        datapod::Pose world_pose_;      // Updated from simulator state
        float linear_velocity_ = 0.0f;  // Forward velocity along heading
        float angular_velocity_ = 0.0f; // Angular velocity (yaw rate)

        // Navigation enabled flag
        bool navigation_enabled_ = true;

      public:
        ControlManager controls;

        // NOTE: sensor/network/container/power managers removed (agent47 handles comms).

        Machine() = default;
        Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config);

        // Prevent copying (Network has unique_ptr members)
        Machine(const Machine &) = delete;
        Machine &operator=(const Machine &) = delete;

        // Allow moving
        Machine(Machine &&) = default;
        Machine &operator=(Machine &&) = default;

        // Initialize all managers
        void init();

        // Update state from simulator feedback
        void update_state(const types::ser::MachineState &state);

        // Tick/tock pattern
        void tick(float dt);
        void tick(float dt, const types::SensorData &sensor_data); // With simulator sensor data
        void tock();

        // Accessors
        const types::Machine &config() const { return config_; }
        types::Machine &config_mut() { return config_; }
        const types::State &state() const { return state_; }
        types::State &state_mut() { return state_; }
        const std::string &uuid() const { return config_.uuid; }
        const std::string &name() const { return config_.name; }
        const datapod::Pose &world_pose() const { return world_pose_; }
        float linear_velocity() const { return linear_velocity_; }
        float angular_velocity() const { return angular_velocity_; }
        std::shared_ptr<rerun::RecordingStream> rec() const { return rec_; }

        // Navigation control
        void set_navigation_enabled(bool enabled) {
            navigation_enabled_ = enabled;
            controls.set_navigation_enabled(enabled);
        }
        bool is_navigation_enabled() const { return navigation_enabled_; }

        // Convenience: direct access to tracker
        drivekit::Tracker *tracker() { return controls.tracker().tracker(); }
        const drivekit::Tracker *tracker() const { return controls.tracker().tracker(); }

        // Velocity access (alias)
        void get_velocity(float &linear, float &angular) const {
            linear = linear_velocity_;
            angular = angular_velocity_;
        }

        // Position access (alias)
        const datapod::Pose &get_position() const { return world_pose_; }
    };

} // namespace agent
