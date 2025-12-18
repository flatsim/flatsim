#pragma once

#include <memory>
#include <rerun.hpp>

#include "flatsim/types.hpp"

namespace agent {

    class Wheel {
      private:
        types::Wheel config_;
        types::Machine machine_config_; // Parent machine config
        concord::Pose world_pose_;      // Updated from simulator state

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Wheel() = default;
        Wheel(const types::Wheel &config, const types::Machine &machine_config,
              std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Update state from simulator feedback
        void update_state(const types::ser::WheelState &state);

        // Set rerun for visualization
        void set_rerun(std::shared_ptr<rerun::RecordingStream> rec) { rec_ = rec; }

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Wheel &config() const { return config_; }
        const concord::Pose &world_pose() const { return world_pose_; }
    };

} // namespace agent
