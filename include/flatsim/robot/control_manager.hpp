#pragma once

#include "flatsim/types.hpp"
#include <vector>

namespace fs {
    // Forward declaration
    class Robot;

    // ============================================================================
    // ControlManager - Handles movement control and propagation through chains
    // ============================================================================
    class ControlManager {
      private:
        Robot *robot = nullptr;
        std::vector<float> steerings, throttles;
        std::vector<float> steerings_max, throttles_max;
        std::vector<float> steerings_diff, throttles_diff;
        float last_steering_input = 0.0f;

      public:
        ControlManager() = default;

        void init(Robot *r, const RobotInfo &robo);
        void reset_controls();
        void set_angular(float angular);
        void set_linear(float linear);
        void set_angular_as_follower(float angular, const Robot &master);
        void set_linear_as_follower(float linear, const Robot &master);

        const std::vector<float> &get_steerings() const { return steerings; }
        const std::vector<float> &get_throttles() const { return throttles; }
        const std::vector<float> &get_steerings_max() const { return steerings_max; }
        const std::vector<float> &get_throttles_max() const { return throttles_max; }
    };

} // namespace fs
