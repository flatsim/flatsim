#include "flatsim/robot/systems/control.hpp"
#include "flatsim/robot.hpp"
#include "flatsim/utils.hpp"

namespace fs {

void ControlSystem::init(const RobotInfo& robo) {
    steerings.resize(robo.wheels.size(), 0.0f);
    steerings_max = robo.controls.steerings_max;
    steerings_diff = robo.controls.steerings_diff;
    throttles.resize(robo.wheels.size(), 0.0f);
    throttles_max = robo.controls.throttles_max;
    throttles_diff = robo.controls.throttles_diff;
}

void ControlSystem::reset_controls() {
    for (uint i = 0; i < steerings.size(); ++i) {
        steerings[i] = 0.0f;
    }
    for (uint i = 0; i < throttles.size(); ++i) {
        throttles[i] = 0.0f;
    }
}

void ControlSystem::set_angular(float angular) {
    constexpr float in_min = -1.0f, in_max = 1.0f;
    const float sign = (angular < 0.0f ? -1.0f : 1.0f);
    for (size_t i = 0; i < steerings.size(); ++i) {
        float o1 = steerings_max[i] - sign * steerings_diff[i];
        float o2 = -steerings_max[i] + sign * steerings_diff[i];
        steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
    }
    
    // Propagate to followers that have steering capability
    for (Robot* follower : robot->get_connected_followers()) {
        if (follower->has_steering_capability()) {
            follower->set_angular_as_follower(angular, *robot);
        }
    }
}

void ControlSystem::set_linear(float linear) {
    constexpr float in_min = -1.0f, in_max = 1.0f;
    
    // Calculate overall vehicle steering angle for differential control
    float overall_steering = 0.0f;
    for (uint i = 0; i < steerings.size(); ++i) {
        if (std::abs(steerings[i]) > std::abs(overall_steering)) {
            overall_steering = steerings[i];
        }
    }
    
    const float sign = (overall_steering < 0.0f ? -1.0f : 1.0f);
    
    for (uint i = 0; i < throttles.size(); ++i) {
        auto lin_val = linear;
        
        // Apply differential for wheels that are actively steering (existing Ackermann logic)
        if (steerings[i] > 0.0f && robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        } else if (steerings[i] < 0.0f && !robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        }
        
        // Apply throttle differential for ALL wheels (not just non-steering ones)
        if (i < throttles_diff.size() && std::abs(throttles_diff[i]) > 1e-6f && std::abs(overall_steering) > 1e-6f) {
            // Apply per-wheel throttle differential on top of any existing adjustments
            float differential_adjustment = sign * throttles_diff[i];
            lin_val = lin_val * (1.0f + differential_adjustment);
        }
        
        throttles[i] = utils::mapper(lin_val, in_min, in_max, throttles_max[i], -throttles_max[i]);
    }
    
    // Propagate to followers that have throttle capability
    for (Robot* follower : robot->get_connected_followers()) {
        if (follower->has_throttle_capability()) {
            follower->set_linear_as_follower(linear, *robot);
        }
    }
}

void ControlSystem::set_angular_as_follower(float angular, const Robot& master) {
    // Only apply if this robot has steering capability and is actually a follower
    if (!robot->has_steering_capability() || robot->role != RobotRole::FOLLOWER) {
        return;
    }
    
    // Apply the same steering logic as master, but potentially with different scaling
    constexpr float in_min = -1.0f, in_max = 1.0f;
    const float sign = (angular < 0.0f ? -1.0f : 1.0f);
    for (size_t i = 0; i < steerings.size(); ++i) {
        float o1 = steerings_max[i] - sign * steerings_diff[i];
        float o2 = -steerings_max[i] + sign * steerings_diff[i];
        steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
    }
    
    // Continue propagation to any followers this robot might have
    for (Robot* follower : robot->get_connected_followers()) {
        if (follower->has_steering_capability()) {
            follower->set_angular_as_follower(angular, *robot);
        }
    }
}

void ControlSystem::set_linear_as_follower(float linear, const Robot& master) {
    // Only apply if this robot has throttle capability and is actually a follower
    if (!robot->has_throttle_capability() || robot->role != RobotRole::FOLLOWER) {
        return;
    }
    
    // Apply the same throttle logic as master, but potentially with different scaling
    constexpr float in_min = -1.0f, in_max = 1.0f;
    for (uint i = 0; i < throttles.size(); ++i) {
        auto lin_val = linear;
        if (steerings[i] > 0.0f && robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        } else if (steerings[i] < 0.0f && !robot->info.controls.left_side[i]) {
            auto proportion = utils::ackermann_scale(steerings[i], robot->info.bound.size.x);
            lin_val = linear * proportion;
        }
        throttles[i] = utils::mapper(lin_val, in_min, in_max, throttles_max[i], -throttles_max[i]);
    }
    
    // Continue propagation to any followers this robot might have
    for (Robot* follower : robot->get_connected_followers()) {
        if (follower->has_throttle_capability()) {
            follower->set_linear_as_follower(linear, *robot);
        }
    }
}

} // namespace fs