#include "flatsim/agent/control_manager.hpp"

namespace agent {

    void ControlManager::init(const types::Machine *m) {
        machine = m;
        steerings.resize(m->wheels.size(), 0.0f);
        steerings_max = m->controls.steerings_max;
        steerings_diff = m->controls.steerings_diff;
        throttles.resize(m->wheels.size(), 0.0f);
        throttles_max = m->controls.throttles_max;
        throttles_diff = m->controls.throttles_diff;
    }

    void ControlManager::reset_controls() {
        for (uint i = 0; i < steerings.size(); ++i) {
            steerings[i] = 0.0f;
        }
        for (uint i = 0; i < throttles.size(); ++i) {
            throttles[i] = 0.0f;
        }
    }

    void ControlManager::set_angular(float angular) {
        constexpr float in_min = -1.0f, in_max = 1.0f;
        last_steering_input = angular; // Store for differential drive mode
        const float sign = (angular < 0.0f ? -1.0f : 1.0f);
        for (size_t i = 0; i < steerings.size(); ++i) {
            float o1 = steerings_max[i] - sign * steerings_diff[i];
            float o2 = -steerings_max[i] + sign * steerings_diff[i];
            steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
        }
    }

    void ControlManager::set_linear(float linear) {
        constexpr float in_min = -1.0f, in_max = 1.0f;

        // Check if this is differential drive mode (all steerings_max are 0)
        bool is_differential_drive = true;
        for (uint i = 0; i < steerings_max.size(); ++i) {
            if (std::abs(steerings_max[i]) > 1e-6f) {
                is_differential_drive = false;
                break;
            }
        }

        // Calculate steering for differential control
        float steering_for_diff = 0.0f;
        if (is_differential_drive) {
            // Differential drive mode: use steering input directly
            steering_for_diff = last_steering_input;
        } else {
            // Ackermann mode: use wheel steering angles
            for (uint i = 0; i < steerings.size(); ++i) {
                if (std::abs(steerings[i]) > std::abs(steering_for_diff)) {
                    steering_for_diff = steerings[i];
                }
            }
        }

        const float sign = (steering_for_diff < 0.0f ? -1.0f : 1.0f);

        for (uint i = 0; i < throttles.size(); ++i) {
            auto lin_val = linear;

            if (is_differential_drive) {
                // Differential drive: map linear/angular into left/right wheel pairs.
                if (i < throttles_diff.size() && std::abs(throttles_diff[i]) > 1e-6f) {
                    float left_cmd = std::clamp(linear + last_steering_input, in_min, in_max);
                    float right_cmd = std::clamp(linear - last_steering_input, in_min, in_max);
                    lin_val = (throttles_diff[i] < 0.0f) ? left_cmd : right_cmd;
                }
            } else {
                // Ackermann mode: apply existing logic
                bool left_side = (i < machine->controls.left_side.size()) ? machine->controls.left_side[i] : false;
                if (steerings[i] > 0.0f && left_side) {
                    auto proportion = utils::ackermann_scale(steerings[i], machine->bound.size.x);
                    lin_val = linear * proportion;
                } else if (steerings[i] < 0.0f && !left_side) {
                    auto proportion = utils::ackermann_scale(steerings[i], machine->bound.size.x);
                    lin_val = linear * proportion;
                }

                // Apply throttle differential for Ackermann vehicles
                if (i < throttles_diff.size() && std::abs(throttles_diff[i]) > 1e-6f &&
                    std::abs(steering_for_diff) > 1e-6f) {
                    float differential_adjustment = sign * throttles_diff[i];
                    lin_val = lin_val * (1.0f + differential_adjustment);
                }
            }

            // Clamp mixed command to the normalized input range before mapping
            lin_val = std::max(in_min, std::min(in_max, lin_val));
            throttles[i] = utils::mapper(lin_val, in_min, in_max, -throttles_max[i], throttles_max[i]);
        }
    }

    types::WheelControl ControlManager::get_wheel_control() const {
        types::WheelControl ctrl;
        if (machine) {
            ctrl.uuid = machine->uuid;
        }
        ctrl.steering = steerings;
        ctrl.throttle = throttles;
        return ctrl;
    }

} // namespace agent
