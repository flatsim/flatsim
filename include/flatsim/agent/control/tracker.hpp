#pragma once

#include "drivekit.hpp"
#include "flatsim/types.hpp"
#include <memory>
#include <rerun.hpp>

namespace agent {

    class Tracker {
      private:
        std::unique_ptr<drivekit::Tracker> tracker_;
        bool enabled_ = false;
        types::Machine *machine_ = nullptr;
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Tracker() = default;

        void init(types::Machine *machine, drivekit::TrackerType type = drivekit::TrackerType::PID,
                  std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Change controller type without reinitializing constraints
        void set_controller_type(drivekit::TrackerType type);

        void set_enabled(bool enabled) { enabled_ = enabled; }
        bool is_enabled() const { return enabled_; }

        drivekit::Tracker *tracker() { return tracker_.get(); }
        const drivekit::Tracker *tracker() const { return tracker_.get(); }

        // Update control based on current state, returns (linear, angular) velocity command
        std::pair<float, float> update(const datapod::Pose &current_pose, float linear_vel, float angular_vel,
                                       float dt);
    };

} // namespace agent
