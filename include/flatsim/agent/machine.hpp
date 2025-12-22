#pragma once

#include <memory>
#include <string>

#include "flatsim/types.hpp"
#include <rerun.hpp>

namespace agent {

    class Machine {
      private:
        std::shared_ptr<rerun::RecordingStream> rec_;

        types::Machine config_;
        types::State state_;
        concord::Pose world_pose_;      // Updated from simulator state
        float linear_velocity_ = 0.0f;  // Forward velocity along heading
        float angular_velocity_ = 0.0f; // Angular velocity (yaw rate)

      public:
        Machine() = default;
        Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config);

        // Update state from simulator feedback
        void update_state(const types::ser::MachineState &state);

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Machine &config() const { return config_; }
        types::Machine &config_mut() { return config_; }
        const types::State &state() const { return state_; }
        types::State &state_mut() { return state_; }
        const std::string &uuid() const { return config_.uuid; }
        const concord::Pose &world_pose() const { return world_pose_; }
        float linear_velocity() const { return linear_velocity_; }
        float angular_velocity() const { return angular_velocity_; }
        std::shared_ptr<rerun::RecordingStream> rec() const { return rec_; }
    };

} // namespace agent
