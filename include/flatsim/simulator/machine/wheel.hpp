#pragma once

#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    class Wheel {
      private:
        muli::RigidBody *body_ = nullptr;
        muli::MotorJoint *motor_joint_ = nullptr;
        muli::AngleJoint *angle_joint_ = nullptr;
        types::Wheel config_;

        // Rate-limited current values
        float current_steering_ = 0.0f;
        float current_throttle_ = 0.0f;
        float steering_rate_ = 0.52f; // rad/s
        float throttle_rate_ = 2.5f;  // 1/s

        // Cached direction vectors (updated in tick)
        muli::Vec2 forward_ = muli::Vec2(0, 1);
        muli::Vec2 normal_ = muli::Vec2(1, 0);

      public:
        Wheel() = default;
        Wheel(const types::Wheel &config);

        // Create physics body and joints, attach to parent machine body
        void create(muli::World &world, muli::RigidBody *parent_body, const concord::Pose &world_pose,
                    const muli::CollisionFilter &filter, float parent_mass);

        // Destroy physics objects
        void destroy(muli::World &world);

        // Apply steering and throttle (rate-limited)
        void apply_control(float target_steering, float target_throttle, float dt);

        // Apply brake impulse
        void apply_brake(float brake_force);

        // Apply lateral friction and drag forces
        void apply_friction();

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Get state for feedback
        types::ser::WheelState get_state() const;

        // Accessors
        muli::RigidBody *body() const { return body_; }
        const types::Wheel &config() const { return config_; }
        float current_steering() const { return current_steering_; }
        float current_throttle() const { return current_throttle_; }
    };

} // namespace simulator
