#pragma once

#include "flatsim/types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace agent {

    class ControlManager {
      private:
        const types::Machine *machine_ = nullptr;
        std::vector<float> steerings_;
        std::vector<float> throttles_;
        float last_angular_input_ = 0.0f;

        static float mapper(float value, float in_min, float in_max, float out_min, float out_max) {
            return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
        }

        static float ackermann_scale(float steering_angle, double chassis_width) {
            float abs_angle = std::abs(steering_angle);
            if (abs_angle < 1e-6f) {
                return 1.0f;
            }
            float width = static_cast<float>(chassis_width);
            float turn_radius = width / std::tan(abs_angle);
            float scale = turn_radius / (turn_radius + width / 2.0f);
            return std::clamp(scale, 0.5f, 1.0f);
        }

      public:
        ControlManager() = default;

        void init(const types::Machine *machine);

        void set_angular(float angular);
        void set_linear(float linear);

        types::MachineControl get_control() const;
    };

} // namespace agent
