#pragma once

#include "flatsim/types.hpp"

namespace agent {

    class Wheel {
      private:
        types::Wheel config_;
        concord::Pose world_pose_; // Updated from simulator state

      public:
        Wheel() = default;
        Wheel(const types::Wheel &config);

        // Update state from simulator feedback
        void update_state(const types::ser::WheelState &state);

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Wheel &config() const { return config_; }
        const concord::Pose &world_pose() const { return world_pose_; }
    };

} // namespace agent
