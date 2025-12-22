#pragma once

#include "drivekit.hpp"
#include "flatsim/types.hpp"
#include <memory>

namespace agent {

    class Controller {
      private:
        std::unique_ptr<drivekit::Tracker> tracker_;
        bool enabled_ = false;
        types::Machine *machine_ = nullptr;

      public:
        Controller() = default;

        void init(types::Machine *machine, drivekit::TrackerType type = drivekit::TrackerType::PID);

        void set_enabled(bool enabled) { enabled_ = enabled; }
        bool is_enabled() const { return enabled_; }

        drivekit::Tracker *tracker() { return tracker_.get(); }
        const drivekit::Tracker *tracker() const { return tracker_.get(); }

        // Update control based on current state, returns (linear, angular) velocity
        std::pair<float, float> update(const concord::Pose &current_pose, float dt);
    };

} // namespace agent
