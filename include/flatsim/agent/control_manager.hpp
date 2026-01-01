#pragma once

#include "flatsim/agent/control/tracker.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include <memory>
#include <vector>

namespace agent {

    // ============================================================================
    // ControlManager - Handles movement control and propagation through chains
    // ============================================================================
    class ControlManager {
      private:
        const types::Machine *machine = nullptr;
        std::vector<float> steerings, throttles;
        std::vector<float> steerings_max, throttles_max;
        std::vector<float> steerings_diff, throttles_diff;
        float last_steering_input = 0.0f;

        Tracker tracker_;
        bool navigation_enabled_ = true;

      public:
        ControlManager() = default;

        void init(const types::Machine *m, std::shared_ptr<rerun::RecordingStream> rec = nullptr);
        void reset_controls();
        void set_angular(float angular);
        void set_linear(float linear);
        types::WheelControl get_wheel_control() const;

        // Navigation/Tracker access
        Tracker &tracker() { return tracker_; }
        const Tracker &tracker() const { return tracker_; }
        void set_navigation_enabled(bool enabled) { navigation_enabled_ = enabled; }
        bool is_navigation_enabled() const { return navigation_enabled_; }

        // Update navigation (called automatically from tick if enabled)
        void update_navigation(const datapod::Pose &current_pose, float linear_vel, float angular_vel, float dt);

        // Tick/tock pattern
        void tick(const datapod::Pose &current_pose, float linear_vel, float angular_vel, float dt);
        void tock(std::shared_ptr<rerun::RecordingStream> rec);

        const std::vector<float> &get_steerings() const { return steerings; }
        const std::vector<float> &get_throttles() const { return throttles; }
        const std::vector<float> &get_steerings_max() const { return steerings_max; }
        const std::vector<float> &get_throttles_max() const { return throttles_max; }
    };

} // namespace agent
