#pragma once

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
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

      public:
        ControlManager() = default;

        void init(const types::Machine *m);
        void reset_controls();
        void set_angular(float angular);
        void set_linear(float linear);
        types::WheelControl get_wheel_control() const;

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        const std::vector<float> &get_steerings() const { return steerings; }
        const std::vector<float> &get_throttles() const { return throttles; }
        const std::vector<float> &get_steerings_max() const { return steerings_max; }
        const std::vector<float> &get_throttles_max() const { return throttles_max; }
    };

} // namespace agent
